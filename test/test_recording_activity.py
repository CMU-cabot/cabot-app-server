#!/usr/bin/env python

# Copyright (c) 2026  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Run with the app-server ROS dependencies sourced, in an isolated ROS graph.

    CABOT_USE_WEBUI=1 python3 -m unittest discover -s test -p test_recording_activity.py
"""

import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import rclpy
from flask import Flask
import socketio
from std_msgs.msg import Bool


ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'cabot_app_server' / directory) for directory in ('scripts', 'webui')]
os.environ['CABOT_USE_WEBUI'] = '1'
import common  # noqa: E402
import webui_api  # noqa: E402


def tearDownModule():
    rclpy.shutdown()
    common.ros2_thread.join(timeout=5)


class RecordingActivityTest(unittest.TestCase):
    def test_optional_topic_and_expiry_in_last_data(self):
        manager = SimpleNamespace(
            device_status=lambda: SimpleNamespace(json={'level': 'OK'}),
            cabot_system_status=lambda: SimpleNamespace(json={'level': 'Active'}),
        )
        server = SimpleNamespace(
            app=Flask(__name__), sio=socketio.Server(), cabot_manager=manager,
            manage_cabot_char=Mock(), destination_char=Mock(), event_char=Mock(),
        )
        with patch.object(webui_api, 'ParamPoller'), \
                patch.object(webui_api.tour_manager.TourManager, 'load'), \
                patch.object(webui_api, 'start_agent'):
            webui_api.WebUI(server)
        client = server.app.test_client()

        def state():
            response = client.get('/api/last_data/')
            self.assertEqual(response.status_code, 200)
            return response.get_json()['recording_reduced_rate']

        # No publisher is required for startup or any API request.
        self.assertIs(state(), False)
        self.assertEqual(client.get('/api/health/').status_code, 200)
        node = rclpy.create_node('recording_activity_test')
        publisher = node.create_publisher(Bool, '/recording/reduced_rate', 1)
        try:
            deadline = time.monotonic() + 5
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertGreater(publisher.get_subscription_count(), 0)

            def publish_and_wait(value):
                publisher.publish(Bool(data=value))
                deadline = time.monotonic() + 3
                while state() != value and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertIs(state(), value)

            publish_and_wait(True)
            publish_and_wait(False)
            publish_and_wait(True)
            node.destroy_publisher(publisher)
            time.sleep(3.1)
            self.assertIs(state(), False)

            # A late/restarted publisher can opt back in; no app restart is needed.
            publisher = node.create_publisher(Bool, '/recording/reduced_rate', 1)
            deadline = time.monotonic() + 5
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                time.sleep(0.05)
            publish_and_wait(True)
        finally:
            node.destroy_node()
