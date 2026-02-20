#!/usr/bin/env python

# Copyright (c) 2025  Carnegie Mellon University
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

import common
import threading
import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


class ParamPoller:
    def __init__(
        self,
        target_node_name,
        param_name,
        result_callback,
        timeout_sec=1.0,
    ):
        node = Node("param_get_client")
        self.client = node.create_client(GetParameters, f"{target_node_name}/get_parameters")
        self.param_name = param_name
        self.result_callback = result_callback
        self.timeout_sec = timeout_sec
        self.in_progress = False
        self.future = None
        self.start_time = 0.0

        def spin_thread():
            rclpy.spin(node)

        threading.Thread(target=spin_thread, daemon=True).start()

    def poll(self):
        if self.in_progress:
            if time.monotonic() - self.start_time > self.timeout_sec:
                if self.future:
                    self.future.cancel()
                self.in_progress = False
            return

        if not self.client.service_is_ready():
            return

        request = GetParameters.Request()
        request.names = [self.param_name]

        self.in_progress = True
        self.start_time = time.monotonic()

        self.future = self.client.call_async(request)
        self.future.add_done_callback(self._on_done)

    def _on_done(self, future):
        self.in_progress = False
        try:
            result = future.result()
            if result and result.values:
                value = result.values[0]
                self.result_callback(value.double_value)
            else:
                self.result_callback(None)
        except Exception:
            self.result_callback(None)
