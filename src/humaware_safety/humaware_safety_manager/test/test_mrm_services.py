"""Service-handler tests for the safety manager node."""

import pytest
import rclpy
from rclpy.parameter import Parameter

from humaware_msgs.msg import SafetyState
from humaware_msgs.srv import ClearMRM, TriggerMRM
from humaware_safety_manager.safety_manager_node import SafetyManagerNode


@pytest.fixture(scope="module", autouse=True)
def _rclpy_default_context():
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


@pytest.fixture()
def node():
    instance = SafetyManagerNode()
    try:
        yield instance
    finally:
        instance.destroy_node()


@pytest.fixture()
def node_with_estop():
    instance = SafetyManagerNode()
    instance.set_parameters([Parameter("estop_engaged", value=True)])
    try:
        yield instance
    finally:
        instance.destroy_node()


def _trigger_request(reason: str = "test_trigger", requester: str = "tester"):
    request = TriggerMRM.Request()
    request.reason = reason
    request.requester = requester
    return request


def _clear_request(reason: str = "test_clear", requester: str = "tester"):
    request = ClearMRM.Request()
    request.reason = reason
    request.requester = requester
    return request


def test_trigger_mrm_marks_service_state(node):
    response = TriggerMRM.Response()
    result = node._handle_trigger_mrm(_trigger_request("operator_button"), response)

    assert result.accepted is True
    assert "operator_button" in result.message
    assert node._service_mrm_active is True
    assert node._service_mrm_reason == "operator_button"


def test_trigger_mrm_uses_default_reason_when_blank(node):
    response = TriggerMRM.Response()
    result = node._handle_trigger_mrm(_trigger_request(reason=""), response)

    assert result.accepted is True
    assert node._service_mrm_reason == "manual_mrm_trigger"
    assert "manual_mrm_trigger" in result.message


def test_clear_mrm_resets_service_state(node):
    pre = TriggerMRM.Response()
    node._handle_trigger_mrm(_trigger_request("preset"), pre)
    assert node._service_mrm_active is True

    response = ClearMRM.Response()
    result = node._handle_clear_mrm(_clear_request("manual"), response)

    assert result.accepted is True
    assert node._service_mrm_active is False
    assert node._service_mrm_reason == ""


def test_clear_mrm_rejected_when_estop_engaged(node_with_estop):
    pre = TriggerMRM.Response()
    node_with_estop._handle_trigger_mrm(_trigger_request("preset"), pre)
    assert node_with_estop._service_mrm_active is True

    response = ClearMRM.Response()
    result = node_with_estop._handle_clear_mrm(_clear_request("manual"), response)

    assert result.accepted is False
    assert result.active_safety_state == SafetyState.STATE_ESTOP
    assert "E-stop" in result.message
    assert node_with_estop._service_mrm_active is True
