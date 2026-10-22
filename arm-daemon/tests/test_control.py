from ctrl import ControlState


def test_starts_human(tmp_path):
    c = ControlState("k", tmp_path / "STOP")
    assert c.holder == "human"


def test_agent_cannot_take(tmp_path):
    c = ControlState("k", tmp_path / "STOP")
    assert c.set_holder("agent", human=False) == (False, "human key required")
    assert c.holder == "human"


def test_human_gives_and_takes(tmp_path):
    c = ControlState("k", tmp_path / "STOP")
    assert c.set_holder("agent", human=True) == (True, "")
    assert c.can_write(human=False) == (True, "")
    assert c.set_holder("human", human=True) == (True, "")
    assert c.can_write(human=False) == (False, "human has control")


def test_human_writes_while_agent_holds(tmp_path):
    c = ControlState("k", tmp_path / "STOP")
    c.set_holder("agent", True)
    assert c.can_write(True) == (True, "")


def test_key_check(tmp_path):
    c = ControlState("secret", tmp_path / "STOP")
    assert c.is_human("secret")
    assert not c.is_human("wrong")
    assert not c.is_human(None)


def test_unknown_holder(tmp_path):
    c = ControlState("k", tmp_path / "STOP")
    assert c.set_holder("robot", human=True) == (False, "unknown holder")


def test_stop_blocks_everyone_until_human_resume(tmp_path):
    c = ControlState("k", tmp_path / "STOP")
    c.stop("test", by="agent")
    assert c.can_write(True) == (False, "stop is set")
    assert c.resume(human=False) == (False, "human key required")
    assert c.resume(human=True) == (True, "")
    assert c.can_write(True) == (True, "")


def test_stop_file_is_honoured_and_cleared(tmp_path):
    f = tmp_path / "STOP"
    c = ControlState("k", f)
    f.touch()
    assert c.stop_state()["by"] == "file"
    assert c.stop_state()["set"] is True
    c.resume(human=True)
    assert not f.exists()
    assert c.stop_state()["set"] is False


def test_released_holder_blocks_all_writes(tmp_path):
    c = ControlState("k", tmp_path / "STOP")
    assert c.set_holder("released", human=False) == (False, "human key required")
    assert c.set_holder("released", human=True) == (True, "")
    assert c.can_write(True) == (False, "bus released")
    assert c.can_write(False) == (False, "bus released")
    assert c.set_holder("human", human=True) == (True, "") and c.can_write(True) == (True, "")


def test_non_ascii_key_is_not_human(tmp_path):
    c = ControlState("k", tmp_path / "STOP")
    assert c.is_human("ключ") is False
    assert c.is_human("k") is True


def test_leader_holder_blocks_writes(tmp_path):
    c = ControlState("k", tmp_path / "STOP")
    assert c.set_holder("leader", human=True) == (True, "")
    assert c.can_write(True) == (False, "leader has control")
    assert c.can_write(False) == (False, "leader has control")
