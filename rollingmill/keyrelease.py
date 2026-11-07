"""Portable terminal key events using the Kitty keyboard protocol."""
import curses
import re
import sys
import time
from collections import deque


class HeldKeys:
    # Kitty functional-key codepoints (not Unicode characters).
    SPECIAL = {57350: curses.KEY_LEFT, 57351: curses.KEY_RIGHT,
               57352: curses.KEY_UP, 57353: curses.KEY_DOWN,
               57348: curses.KEY_IC, 57349: curses.KEY_DC,
               57354: curses.KEY_PPAGE, 57355: curses.KEY_NPAGE,
               57356: curses.KEY_HOME, 57357: curses.KEY_END,
               127: curses.KEY_BACKSPACE}
    JOG = {curses.KEY_RIGHT: ('X', 1), curses.KEY_LEFT: ('X', -1),
           curses.KEY_UP: ('Y', 1), curses.KEY_DOWN: ('Y', -1),
           ord('a'): ('Z', 1), ord('A'): ('Z', 1),
           ord('z'): ('Z', -1), ord('Z'): ('Z', -1),
           ord(']'): ('A', 1), ord('['): ('A', -1)}

    def __init__(self):
        self.supported = False
        self._held = set()
        self._buffer = ''
        self._pending = deque()
        self._escape_at = 0.0
        self._started = False
        self.event_time = None
        self.event_type = 1

    def _queue(self, key, event=1):
        self._pending.append((key, time.monotonic(), event))

    def _pop(self):
        key, self.event_time, self.event_type = self._pending.popleft()
        return key

    @property
    def window(self):
        # A compatibility token for TUI's current input session.
        return 1 if self.supported else None

    def start(self):
        if not self._started:
            # Disambiguate escape codes + report press/repeat/release; ask for
            # a response rather than trusting TERM or guessing key-up timing.
            sys.stdout.write('\x1b[>11u\x1b[?u\x1b[?1004h')
            sys.stdout.flush()
            self._started = True

    def feed(self, data):
        self._buffer += data
        while self._buffer:
            if not self._buffer.startswith('\x1b'):
                self._queue(ord(self._buffer[0]))
                self._buffer = self._buffer[1:]
                continue
            if self._buffer == '\x1b' or self._buffer == '\x1b[':
                break
            match = re.match(r'\x1b\[([0-9;:?<>]*)([A-Za-z~])', self._buffer)
            if not match:
                if len(self._buffer) > 64 or not self._buffer.startswith('\x1b['):
                    self._queue(27)
                    self._buffer = self._buffer[1:]
                    continue
                break
            args, final = match.groups()
            self._buffer = self._buffer[match.end():]
            if final == 'u' and args.startswith('?'):
                self.supported = (int(args[1:] or '0') & 11) == 11
            elif final == 'u' and not args.startswith(('>', '<')):
                fields = args.split(';')
                code = int(fields[0].split(':')[0])
                key = self.SPECIAL.get(code, code)
                event = 1
                if len(fields) > 1 and ':' in fields[1]:
                    event = int(fields[1].split(':')[1])
                jog = self.JOG.get(key)
                if event == 3:
                    if jog:
                        self._held.discard(jog)
                elif event in (1, 2):
                    if jog:
                        self._held.add(jog)
                    self._queue(key, event)
            elif final == 'O':  # Focus out, CSI O (focus in = CSI I).
                self._held.clear()
            elif final in 'ABCDHF' or final == '~':
                key = {'A': curses.KEY_UP, 'B': curses.KEY_DOWN,
                       'C': curses.KEY_RIGHT, 'D': curses.KEY_LEFT,
                       'H': curses.KEY_HOME, 'F': curses.KEY_END}.get(final)
                if final == '~':
                    key = {'2': curses.KEY_IC, '3': curses.KEY_DC,
                           '5': curses.KEY_PPAGE, '6': curses.KEY_NPAGE}.get(args.split(';')[0])
                fields = args.split(';')
                event = int(fields[1].split(':')[1]) if len(fields) > 1 and ':' in fields[1] else 1
                jog = self.JOG.get(key)
                if event == 3:
                    if jog:
                        self._held.discard(jog)
                elif key is not None:
                    if jog and self.supported:
                        self._held.add(jog)
                    self._queue(key, event)
            elif final == 'Z':
                self._queue(curses.KEY_BTAB)
        if self._buffer and not self._escape_at:
            self._escape_at = time.monotonic()
        elif not self._buffer:
            self._escape_at = 0.0

    def read(self, screen):
        if self._pending:
            return self._pop()
        key = screen.getch()
        if 0 <= key < 256:
            self.feed(chr(key))
            # Drain the rest of an encoded event without a timeout per byte.
            screen.timeout(0)
            try:
                while True:
                    key = screen.getch()
                    if key == -1:
                        break
                    if 0 <= key < 256:
                        self.feed(chr(key))
                    else:
                        self._queue(key)
            finally:
                screen.timeout(25)
        elif key != -1:
            self._queue(key)
        if self._buffer == '\x1b' and time.monotonic() - self._escape_at >= 0.025:
            self._buffer = ''
            self._escape_at = 0.0
            self._queue(27)
        return self._pop() if self._pending else -1

    def held(self, axis, sign, window):
        return bool(self.supported and (axis, sign) in self._held
                    and (axis, -sign) not in self._held)

    def suspend(self):
        self._held.clear()
        if self._started:
            sys.stdout.write('\x1b[<u\x1b[?1004l')
            sys.stdout.flush()
            self._started = False

    def close(self):
        self.suspend()
