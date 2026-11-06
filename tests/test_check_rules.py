from rollingmill.cutjob import (
    RULE_G_DECIMAL, RULE_G_ONE_DIGIT, RULE_G_UNSUPPORTED, RULE_INLINE_COMMENT,
    RULE_M_UNSUPPORTED, RULE_PAREN_EMBEDDED, RULE_SPINDLE_OFF,
    check_nc, parse_nc_blocks,
)


def rules(text):
    return [(w.rule, w.line, w.column) for w in check_nc(parse_nc_blocks(text.encode()))]


def test_clean_program():
    assert rules("G21\nS10000\nM03\nG00 Z5.0\nG01 X1.0 F100.0\nM05\n") == []


def test_one_digit_g_code():
    assert rules("M03\nG1 X1.0\n") == [(RULE_G_ONE_DIGIT, 2, 1)]
    assert rules("M03\nG01 X1.0 G4 P1\n") == [(RULE_G_ONE_DIGIT, 2, 10)]


def test_g_rule_ignores_comments_and_other_words():
    assert rules("M03\n(G1 here)\nG01 X1.0 F5.0\n; G1\nG01 Z-1.0\n") == []


def test_spindle_off_before_cut():
    assert rules("G00 Z5.0\nG01 Z-1.0\nG01 X1.0\n") == [(RULE_SPINDLE_OFF, 2, 1)]
    assert rules("M03\nG01 X1.0\nM05\nG02 X1.0 I1.0\nG03 X2.0 I1.0\n") == [(RULE_SPINDLE_OFF, 4, 1)]


def test_rapids_without_spindle_ok():
    assert rules("G00 X1.0\nM03\nG01 X2.0\n") == []


def test_paren_rule_still_fires():
    assert rules("(a (b)\n")[0][0] == RULE_PAREN_EMBEDDED


def test_packed_words_without_spaces():
    assert rules("F300.0S6000M03\nG17G41D01G00X8.0Y8.0\nG01Z-7.0\n") == []
    assert rules("M03\nG17G1X1.0\n") == [(RULE_G_ONE_DIGIT, 2, 4)]


def test_inline_comment():
    assert rules("M03\nG01 X1.0 (note) F5.0\n") == [(RULE_INLINE_COMMENT, 2, 10)]
    assert rules("M03\nG01 X1.0 (unclosed\n") == [(RULE_INLINE_COMMENT, 2, 10)]


def test_own_line_comment_ok():
    assert rules("M03\n(note)\n  (indented note)\nG01 X1.0\n") == []


def test_decimal_g_code():
    assert rules("M03\nG38.2 Z-1.0\n") == [(RULE_G_DECIMAL, 2, 1)]
    assert rules("G17G90.1\n") == [(RULE_G_DECIMAL, 1, 4)]
    assert rules("(G90.1 in a comment)\nM03\nG01 X1.0 F5.0\n") == []


def test_unsupported_g_and_m_codes():
    assert rules("M03\nG64 X1.0\n") == [(RULE_G_UNSUPPORTED, 2, 1)]
    assert rules("G17G94G00\n") == [(RULE_G_UNSUPPORTED, 1, 4)]
    assert rules("M08\nM04\n") == [(RULE_M_UNSUPPORTED, 1, 1), (RULE_M_UNSUPPORTED, 2, 1)]


def test_every_listed_code_accepted():
    from rollingmill.cutjob import SUPPORTED_G, SUPPORTED_M
    prog = "M03\n" + "".join(f"G{g:02d}\n" for g in sorted(SUPPORTED_G))
    prog += "".join(f"M{m:02d}\n" for m in sorted(SUPPORTED_M - {5, 2, 30}))
    got = [r for r in rules(prog) if r[0] in (RULE_G_UNSUPPORTED, RULE_M_UNSUPPORTED)]
    assert got == []


def test_m04_no_longer_starts_spindle():
    got = [r[0] for r in rules("M04\nG01 X1.0\n")]
    assert got == [RULE_M_UNSUPPORTED, RULE_SPINDLE_OFF]
