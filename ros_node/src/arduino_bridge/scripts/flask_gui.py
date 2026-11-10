#!/usr/bin/env python3
"""Basic Flask controls and sensor display over the two Arduino ROS topics."""
#TODO add input validation

import argparse
import copy
import math
from threading import Lock, Thread

from flask import Flask, jsonify, render_template, request
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def number(value):
    """Accept a finite number representable by the Uno's float."""
    if isinstance(value, bool):
        raise ValueError('Enter a finite number.')
    value = float(value)
    if not math.isfinite(value) or abs(value) > 3.402823466e38:
        raise ValueError('Enter a finite number within the Arduino float range.')
    return value


class MotorGui(Node):
    def __init__(self, step_degrees=None):
        super().__init__('motor_gui')
        self.step_degrees = step_degrees
        self.lock = Lock()
        self.state = {
            'mode': None, 'enabled': None,
            'pot': None, 'flex': None, 'tof_mm': None, 'slot': None,
            'dc_command': None, 'servo_command': None, 'stepper_command': None,
            'step_degrees': step_degrees,
        }
        self.publisher = self.create_publisher(String, '/arduino/command', 10)
        self.subscription = self.create_subscription(
            String, '/arduino/data', self.receive_data, 10)

    def receive_data(self, msg):
        fields = msg.data.strip().split(',')
        try:
            tag, *values = fields
            if tag == 'POT' and len(values) == 2:
                key, value = 'pot', {'raw_adc': int(values[0]),
                                     'angle_deg': number(values[1])}
            elif tag == 'FLEX' and len(values) == 3:
                key, value = 'flex', {'raw_adc': int(values[0]),
                                      'filtered_adc': number(values[1]),
                                      'angle_deg': number(values[2])}
            elif tag == 'TOF_MM' and len(values) == 1:
                key, value = 'tof_mm', int(values[0])
            elif tag == 'SLOT' and len(values) == 2 and values[0] in ('0', '1'):
                key, value = 'slot', {'blocked': values[0] == '1',
                                      'count': int(values[1])}
            elif tag == 'ENABLE' and len(values) == 1 and values[0] in ('0', '1'):
                key, value = 'enabled', values[0] == '1'
            else:
                return
        except (ValueError, OverflowError):
            return
        with self.lock:
            self.state[key] = value

    def send_command(self, command, argument):
        with self.lock:
            if command == 'MODE':
                if argument not in ('GUI', 'SENSOR'):
                    raise ValueError('Mode must be GUI or SENSOR.')
                key, value = 'mode', argument
            elif command == 'ENABLE':
                if argument not in ('0', '1'):
                    raise ValueError('Enable must be 0 or 1.')
                key, value = 'enabled', argument == '1'
            elif command in ('DC_MOVE', 'DC_VEL', 'SERVO', 'STEPPER_MOVE'):
                if self.state['mode'] != 'GUI' or self.state['enabled'] is not True:
                    raise ValueError('Select GUI mode and enable motors first.')
                value = number(argument)
                if command == 'STEPPER_MOVE':
                    if self.step_degrees is None:
                        raise ValueError('Start the GUI with confirmed --step-degrees.')
                    steps = math.floor(abs(value) / self.step_degrees + 0.5)
                    value = number(math.copysign(steps * self.step_degrees, value))
                argument = format(value, '.9g')
                key = {'DC_MOVE': 'dc_command', 'DC_VEL': 'dc_command',
                       'SERVO': 'servo_command',
                       'STEPPER_MOVE': 'stepper_command'}[command]
                value = {'command': command, 'value': value}
            else:
                raise ValueError('Unknown command.')
            self.publisher.publish(String(data=f'{command},{argument}'))
            self.state[key] = value
        return f'{command},{argument}'


def create_app(node):
    app = Flask(__name__)

    @app.get('/')
    def index():
        return render_template('index.html')

    @app.get('/state')
    def state():
        with node.lock:
            result = copy.deepcopy(node.state)
        return jsonify(result)

    @app.post('/command')
    def command():
        data = request.get_json(silent=True)
        if not isinstance(data, dict):
            return jsonify(error='Expected a JSON command and value.'), 400
        try:
            sent = node.send_command(data.get('command'), data.get('value'))
        except (ValueError, TypeError, OverflowError) as exc:
            return jsonify(error=str(exc)), 400
        return jsonify(sent=sent)

    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--step-degrees', type=float,
                        help='Confirmed degrees per driver pulse, including microstepping.')
    args = parser.parse_args()
    if args.step_degrees is not None and (
            not math.isfinite(args.step_degrees) or args.step_degrees <= 0):
        parser.error('--step-degrees must be finite and positive')
    rclpy.init()
    node = MotorGui(args.step_degrees)
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    try:
        create_app(node).run(host='127.0.0.1', port=5000,
                             debug=False, use_reloader=False, threaded=False)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join()
        node.destroy_node()


if __name__ == '__main__':
    main()
