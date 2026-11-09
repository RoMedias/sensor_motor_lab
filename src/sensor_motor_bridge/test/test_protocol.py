from sensor_motor_bridge import protocol as P


def test_header_and_telemetry():
    kind, header = P.parse_line("H,mode,sensors/pot_deg,motors/dc_rpm\r\n")
    assert kind == "H" and header == ["mode", "sensors/pot_deg", "motors/dc_rpm"]
    kind, vals = P.parse_line("T,1,135.5,-12.25\r\n")
    assert kind == "T"
    assert P.decode_telemetry(header, vals) == {
        "mode": 1.0, "sensors/pot_deg": 135.5, "motors/dc_rpm": -12.25}


def test_count_mismatch_rejected():
    _, vals = P.parse_line("T,1,135.5")
    assert P.decode_telemetry(["a", "b", "c"], vals) is None
    assert P.decode_telemetry(None, vals) is None


def test_garbage_rejected():
    assert P.parse_line("T,1,13x.5") == (None, None)
    assert P.parse_line("H,bad name,ok") == (None, None)
    assert P.parse_line("H,9starts_with_digit") == (None, None)
    assert P.parse_line("\x00\xffjunk") == (None, None)
    assert P.parse_line("") == (None, None)


def test_ack_and_error():
    assert P.parse_line("A,servo") == ("A", "servo")
    assert P.parse_line("E,ignored: SENSOR mode") == ("E", "ignored: SENSOR mode")


def test_commands():
    assert P.cmd_stepper_rel(10) == "SR 10.000\n"
    assert P.cmd_mode(True) == "M 1\n"
    assert P.cmd_request_header() == "?\n"
