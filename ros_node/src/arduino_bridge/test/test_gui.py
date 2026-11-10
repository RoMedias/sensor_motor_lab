"""Exercise HTTP commands and telemetry without a serial port or ROS discovery."""

from pathlib import Path
import sys
from unittest.mock import Mock, patch

import pytest
from rclpy.node import Node
from std_msgs.msg import String

# Import the runnable script directly from the source checkout.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from flask_gui import create_app, MotorGui  # noqa: E402


@pytest.fixture
def gui():
    with patch.object(Node, '__init__', return_value=None), \
            patch.object(Node, 'create_publisher', return_value=Mock()), \
            patch.object(Node, 'create_subscription', return_value=Mock()):
        node = MotorGui(step_degrees=0.25)  # Test fixture, not a hardware setting.
    return node, create_app(node).test_client()


def test_mode_enable_and_physical_button_gate_motor_commands(gui):
    node, client = gui
    command = {'command': 'DC_MOVE', 'value': '45'}
    assert client.post('/command', json=command).status_code == 400
    client.post('/command', json={'command': 'MODE', 'value': 'GUI'})
    client.post('/command', json={'command': 'ENABLE', 'value': '1'})
    assert client.post('/command', json=command).json['sent'] == 'DC_MOVE,45'
    node.receive_data(String(data='ENABLE,0'))
    assert client.get('/state').json['enabled'] is False
    assert client.post('/command', json=command).status_code == 400
    node.receive_data(String(data='ENABLE,1'))
    assert client.get('/state').json['enabled'] is True
    assert client.post('/command', json=command).status_code == 200
    client.post('/command', json={'command': 'ENABLE', 'value': '1'})
    client.post('/command', json={'command': 'MODE', 'value': 'SENSOR'})
    assert client.post('/command', json=command).status_code == 400


def test_dc_reset_is_gated_and_publishes_stop_reset_command(gui):
    node, client = gui
    reset = {'command': 'DC_RESET', 'value': '0'}
    assert client.post('/command', json=reset).status_code == 400
    client.post('/command', json={'command': 'MODE', 'value': 'GUI'})
    client.post('/command', json={'command': 'ENABLE', 'value': '1'})
    assert client.post('/command', json=reset).json['sent'] == 'DC_RESET,0'
    assert node.publisher.publish.call_args.args[0].data == 'DC_RESET,0'
    assert client.get('/state').json['dc_command'] == {
        'command': 'DC_RESET', 'value': 0}
    node.publisher.publish.reset_mock()
    assert client.post('/command', json={
        'command': 'DC_RESET', 'value': '1'}).status_code == 400
    node.receive_data(String(data='ENABLE,0'))
    assert client.post('/command', json=reset).status_code == 400
    node.publisher.publish.assert_not_called()
    client.post('/command', json={'command': 'ENABLE', 'value': '1'})
    client.post('/command', json={'command': 'MODE', 'value': 'SENSOR'})
    node.publisher.publish.reset_mock()
    assert client.post('/command', json=reset).status_code == 400
    node.publisher.publish.assert_not_called()


@pytest.mark.parametrize('requested,rounded', [
    ('0.125', 0.25), ('-0.125', -0.25), ('0.1', 0), ('-45.1', -45)])
def test_stepper_publishes_and_displays_achievable_command(gui, requested, rounded):
    node, client = gui
    client.post('/command', json={'command': 'MODE', 'value': 'GUI'})
    client.post('/command', json={'command': 'ENABLE', 'value': '1'})
    response = client.post('/command', json={
        'command': 'STEPPER_MOVE', 'value': requested})
    assert response.status_code == 200
    sent = node.publisher.publish.call_args.args[0].data
    assert float(sent.split(',')[1]) == rounded
    assert client.get('/state').json['stepper_command']['value'] == rounded


def test_unconfigured_stepper_and_invalid_commands_are_rejected(gui):
    node, client = gui
    client.post('/command', json={'command': 'MODE', 'value': 'GUI'})
    client.post('/command', json={'command': 'ENABLE', 'value': '1'})
    node.step_degrees = None
    node.publisher.publish.reset_mock()
    for command, value in [('STEPPER_MOVE', '10'), ('SERVO', 'nan'),
                           ('DC_VEL', 'inf'), ('DC_MOVE', '45\nENABLE,1'),
                           ('DC_MOVE', '1e100'), ('MODE', 'OTHER')]:
        assert client.post('/command', json={'command': command,
                                             'value': value}).status_code == 400
    node.publisher.publish.assert_not_called()


def test_sensor_values_preserve_raw_filtered_and_physical_units(gui):
    node, client = gui
    for line in ('POT,512,90', 'FLEX,620,605.25,35', 'TOF_MM,-1', 'SLOT,1,12'):
        node.receive_data(String(data=line))
    state = client.get('/state').json
    assert state['pot'] == {'raw_adc': 512, 'angle_deg': 90}
    assert state['flex'] == {'raw_adc': 620, 'filtered_adc': 605.25, 'angle_deg': 35}
    assert state['tof_mm'] == -1
    assert state['slot'] == {'blocked': True, 'count': 12}
    for malformed in ('POT,hello,90', 'FLEX,620,nan,35', 'SLOT,yes,12'):
        node.receive_data(String(data=malformed))
    assert client.get('/state').json == state
    assert client.get('/').status_code == 200
