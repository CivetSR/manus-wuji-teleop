from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_cpp_glove_topics_use_stable_id_to_index_binding() -> None:
    source = (ROOT / "manus/ros2/src/ManusDataPublisher.cpp").read_text()
    assert "m_GloveTopicIndex.emplace(t_Msg.glove_id, topic_index)" in source
    assert "m_GloveTopicIndex.find(glove_id)" in source
    assert "std::distance(m_GlovePublisher.begin()" not in source


def test_cpp_node_info_lookup_checks_bounds_before_dereference() -> None:
    source = (ROOT / "manus/ros2/src/ManusDataPublisher.cpp").read_text()
    bounds_check = source.index("if (t_NodeInfoIndex == m_NodeInfoCount)")
    dereference = source.index(
        "t_Node.parent_node_id = m_NodeInfo[t_NodeInfoIndex].parentId"
    )
    assert bounds_check < dereference


def test_python_vibration_topic_comes_from_subscribed_topic_index() -> None:
    source = (ROOT / "bridge/x86/manus_wuji_bridge.py").read_text()
    assert 're.fullmatch(r"/manus_glove_(\\d+)", source_topic)' in source
    assert "index = len(self.vib_publishers)" not in source


def test_dynamic_glove_subscriptions_are_retained_explicitly() -> None:
    source = (ROOT / "bridge/x86/manus_wuji_bridge.py").read_text()
    assert "self._glove_subscriptions[topic] = subscription" in source


def test_status_reports_last_drop_reason_and_side_mismatch_command() -> None:
    source = (ROOT / "bridge/x86/manus_wuji_bridge.py").read_text()
    assert "last_drop={last_drop}" in source
    assert "dropped_side_mismatch=" in source
    assert "HEADLESS=0 SIDES={side}" in source
