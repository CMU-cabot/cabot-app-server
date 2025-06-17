#!/usr/bin/env python

# Copyright (c) 2022  Carnegie Mellon University
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

import ctypes
import asyncio
import os
import time
import json
import math
import threading
import traceback
import signal
import subprocess
import sys
from rosidl_runtime_py.convert import message_to_ordereddict

import rclpy

import common
import ble
import tcp

from cabot_common import util
from cabot_log_report import LogReport
from cabot_msgs.srv import Speak
from std_srvs.srv import Trigger, SetBool
import sensor_msgs.msg
import power_controller_msgs.msg

MTU_SIZE = 2**10  # could be 2**15, but 2**10 is enough
CHAR_WRITE_MAX_SIZE = 512  # should not be exceeded this value
WAIT_AFTER_CONNECTION = 0.25  # wait a bit after connection to avoid error


class DeviceStatus:
    def __init__(self):
        self.level = "Unknown"
        self.devices = []
        self.temperatures = {}

    def ok(self):
        self.level = "OK"

    def error(self):
        self.level = "Error"

    def set_json(self, text):
        try:
            self.devices = []
            data = json.loads(text)
            if 'devices' in data:
                for dev in data['devices']:
                    device = {
                        'type': dev['device_type'],
                        'model': dev['device_model'],
                        'level': "OK" if dev['device_status'] == "0" else "Error",
                        'message': dev['device_message'],
                        'values': []
                    }
                    for key in dev:
                        device['values'].append({
                            'key': key,
                            'value': dev[key]
                        })
                    self.devices.append(device)
        except:  # noqa: E722
            common.logger.info(traceback.format_exc())

    def set_wifi_status(self, result):
        device = {
            "type": "WiFi",
            "model": "Unknown",
            "level": "Error",
            "message": "not_found",
            "values": []
        }
        if result and result.returncode == 0:
            if "Soft blocked: no" in result.stdout:
                device = {
                    "type": "WiFi",
                    "model": "Unknown",
                    "level": "OK",
                    "message": "enabled",
                    "values": []
                }
            elif "Soft blocked: yes" in result.stdout:
                device = {
                    "type": "WiFi",
                    "model": "Unknown",
                    "level": "OK",
                    "message": "disabled",
                    "values": []
                }
        self.devices.append(device)

    def set_clients(self, clients):
        device = {
            'type': "User App",
            'model': "cabot-ios-app",
            'level': "Error",
            'message': "disconnected",
            'values': []
        }
        for client in clients:
            if not client.connected:
                continue
            device = {
                'type': "User App",
                'model': "cabot-ios-app",
                'level': "OK",
                'message': "connected",
                'values': []
            }
            break
        self.devices.append(device)

    def set_temperature_status(self):
        for temperature_data in self.temperatures.values():
            if temperature_data is None:
                temperature_value = ""
                temperature_location = "Unknown"
                temperature_level = "Unknown"
            else:
                temperature, frame_id = temperature_data
                temperature_value = f"{int(temperature)}℃"
                temperature_level = "Error" if temperature > 50 else "OK"
                temperature_location = frame_id if frame_id else "Unknown"
            device = {
                'type': "Suitcase Temperature",
                'model': temperature_location,
                'level': temperature_level,
                'message': temperature_value,
                'values': []
            }
            self.devices.append(device)

    @property
    def json(self):
        return {
            'level': self.level,
            'devices': self.devices
        }

    def stop(self):
        pass


class SystemStatus:
    def __init__(self):
        self.level = "Unknown"
        self.diagnostics = []

    def activating(self):
        self.level = "Activating"

    def deactivating(self):
        self.level = "Deactivating"

    def active(self):
        self.level = "Active"

    def inactive(self):
        self.level = "Inactive"

    def error(self):
        self.level = "Error"

    def set_diagnostics(self, diagnostics):
        self.diagnostics = diagnostics

    def is_active(self):
        if self.level == "Active":
            return True
        if len(self.diagnostics) > 0:
            return True
        return False

    @property
    def json(self):
        return {
            'level': self.level,
            'diagnostics': self.diagnostics
        }

    def stop(self):
        self.deactivating()
        self.diagnostics = []


class AppClient():
    ALIVE_THRESHOLD = 3.0

    def __init__(self, id):
        self.client_id = id
        self.type = None
        self.last_updated = time.time()

    @property
    def connected(self):
        return time.time() - self.last_updated < AppClient.ALIVE_THRESHOLD

    def __str__(self):
        return f"AppClient: client_id={self.client_id}, type={self.type}"


class CaBotManager():
    def __init__(self):
        self._device_status = DeviceStatus()
        self._cabot_system_status = SystemStatus()
        self._battery_status = None
        self._battery_states = None
        self._log_report = LogReport()
        self.systemctl_lock = threading.Lock()
        self.start_flag = False
        self.stop_run = None
        self.check_interval = 5
        self.run_count = 0
        self._client_map = {}

    def run(self, start=False):
        self.start_flag = start
        self._run_once()
        self.stop_run = self._run()

    def stop(self):
        if self.stop_run:
            self.stop_run.set()

    def temperature_states(self, topic_name, msg):
        self._device_status.temperatures[topic_name] = (msg.temperature, msg.header.frame_id)

    def battery_states(self, msg):
        self._battery_states = msg

    def battery_state(self, msg):
        class Dummy():
            def __init__(self, msg, states):
                def percent(value):
                    if value >= 0:
                        return "{:.0f}%".format(value * 100)
                    else:
                        return "Unknown"

                level = 0
                if msg.percentage <= 0.2:
                    level = 1
                if msg.percentage <= 0.1:
                    level = 2
                self._json = {
                    'name': "Battery Control",
                    'level': level,
                    'message': percent(msg.percentage),
                    'hardware_id': msg.header.frame_id,
                    'values': [{
                        'key': 'Battery Percentage',
                        'value': percent(msg.percentage)
                    }]
                }
                if states:
                    for b in states.batteryarray:
                        if not math.isnan(b.percentage):
                            self._json['values'].append({
                                'key': f"Battery {b.location} ({b.serial_number})",
                                'value': percent(b.percentage),
                            })
                        else:
                            self._json['values'].append({
                                'key': f"Battery {b.location}",
                                'value': "not available",
                            })
                common.logger.info(f"{self._json}")

            @property
            def json(self):
                return self._json
        self._battery_status = Dummy(msg, self._battery_states)

    def add_log_request(self, request, callback, output=True):
        self._log_report.add_to_queue(request, callback, output)

    @util.setInterval(1)
    def _run(self):
        self._run_once()

    def _run_once(self):
        self.run_count += 1
        if self.check_interval <= self.run_count:
            self._check_device_status()
            self._check_service_active()
            self.run_count = 0

        if self.start_flag:
            if self._device_status.level == "OK":
                self.start_flag = False
                if self._cabot_system_status.level != "Active":
                    self.start()
            else:
                common.logger.info("Start at launch is requested, but device is not OK")

    def _check_device_status(self):
        if not os.path.exists('/opt/cabot-device-check/check_device_status.sh'):
            self._device_status.ok()   # TODO: work around for cabot3-k4, not implemented
            self._device_status.set_json("{}")
            self._device_status.set_wifi_status(self._runprocess(["rfkill", "list", "wifi"]))
            self._device_status.set_clients(self.get_clients_by_type("Normal"))
            self._device_status.set_temperature_status()
            return
        if self._cabot_system_status.is_active():
            result = self._runprocess(["sudo", "-E", "/opt/cabot-device-check/check_device_status.sh", "-j", "-s"])
        else:
            result = self._runprocess(["sudo", "-E", "/opt/cabot-device-check/check_device_status.sh", "-j"])
        if result and result.returncode == 0:
            self._device_status.ok()
        else:
            self._device_status.error()
        self._device_status.set_json(result.stdout)
        self._device_status.set_wifi_status(self._runprocess(["rfkill", "list", "wifi"]))
        self._device_status.set_clients(self.get_clients_by_type("Normal"))
        self._device_status.set_temperature_status()

    def _check_service_active(self):
        result = self._runprocess(["systemctl", "--user", "is-active", "cabot"])
        common.logger.info(f"_check_service_active {result}")
        if not result:
            return
        if result.returncode == 0:
            self._cabot_system_status.active()
        else:
            if result.stdout.strip() == "inactive":
                self._cabot_system_status.inactive()
            elif result.stdout.strip() == "failed":
                self._cabot_system_status.inactive()
            elif result.stdout.strip() == "deactivating":
                self._cabot_system_status.deactivating()
            else:
                common.logger.info("check_service_active unknown status: %s", result.stdout.strip())

        # global diagnostics
        self._cabot_system_status.set_diagnostics(common.diagnostics)
        common.diagnostics = []

    def _runprocess(self, command):
        return subprocess.run(command, capture_output=True, text=True, env=os.environ.copy())

    def _call(self, command, lock=None):
        result = 0
        if lock is not None and not lock.acquire(blocking=False):
            common.logger.info("lock could not be acquired")
            return result
        try:
            common.logger.info("calling %s", str(command))
            result = subprocess.call(command)
        except:  # noqa: E722
            common.logger.error(traceback.format_exc())
        finally:
            if lock is not None:
                lock.release()
        return result

    def rebootPC(self):
        self._call(["sudo", "systemctl", "reboot"], lock=self.systemctl_lock)

    def poweroffPC(self):
        if shutdown_client.wait_for_service(timeout_sec=5.0):
            req = Trigger.Request()
            shutdown_client.call(req)

    def releaseEmergencystop(self, release=True):
        if set_24v_power_odrive_client.wait_for_service(timeout_sec=5.0):
            req = SetBool.Request()
            req.data = release
            set_24v_power_odrive_client.call(req)
        else:
            common.logger.error(f"timeout /set_24v_power_odrive service")

    def startCaBot(self):
        self._call(["systemctl", "--user", "start", "cabot"], lock=self.systemctl_lock)
        self._cabot_system_status.activating()

    def stopCaBot(self):
        self._call(["systemctl", "--user", "stop", "cabot"], lock=self.systemctl_lock)
        self._cabot_system_status.deactivating()

    def enableWiFi(self, enable):
        self._call(["rfkill", "unblock" if enable else "block", "wifi"])

    def device_status(self):
        return self._device_status

    def cabot_system_status(self):
        return self._cabot_system_status

    def cabot_battery_status(self):
        return self._battery_status

    def register_client(self, client_id=None, client_type=None):
        if client_id is None:
            return
        client = AppClient(client_id)
        if client_id in self._client_map:
            client = self._client_map[client_id]
        else:
            self._client_map[client_id] = client
        client.last_updated = time.time()
        if client_id is not None and client.client_id is None:
            client.client_id = client_id
            common.logger.info(client)
        if client_type is not None and client.type is None:
            client.type = client_type
            common.logger.info(client)
        return client

    def get_client(self, id):
        if id in self._client_map:
            return self._client_map[id]
        return None

    def get_clients_by_type(self, client_type):
        clients = []
        for key in self._client_map.keys():
            if self._client_map[key].type != client_type:
                continue
            clients.append(self._client_map[key])
        return clients


quit_flag = False
tcp_server = None
ble_manager = None
tcp_server_thread = None


def get_thread_traceback(thread_id):
    frame = sys._current_frames().get(thread_id)
    if frame:
        return ''.join(traceback.format_stack(frame))
    else:
        return "Thread not found"


def terminate_thread(thread):
    if not thread.is_alive():
        return
    exc = ctypes.py_object(SystemExit)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread.ident), exc)
    if res == 0:
        raise ValueError("Invalid thread ID")
    elif res > 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, 0)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def sigint_handler(sig, frame):
    common.logger.info("sigint_handler")
    global quit_flag
    if sig == signal.SIGINT:
        try:
            quit_flag = True
            if ble_manager:
                ble_manager.stop()

            if tcp_server_thread:
                try:
                    terminate_thread(tcp_server_thread)
                except:  # noqa: E722
                    common.logger.error(traceback.format_exc())
                while tcp_server_thread.is_alive():
                    common.logger.info(f"wait tcp server thread {tcp_server_thread}")
                    tcp_server_thread.join(timeout=1)

            if common.ros2_thread:
                try:
                    rclpy.shutdown()
                except:  # noqa: E722
                    common.logger.error(traceback.format_exc())
                while common.ros2_thread.is_alive():
                    common.logger.info(f"wait ros2 server thread {common.ros2_thread}")
                    common.ros2_thread.join(timeout=1)
        except:  # noqa: E722
            common.logger.error(traceback.format_exc())
    else:
        common.logger.error("Unexpected signal")


class TemperatureSubscriberManager:
    def __init__(self, node, cabot_manager):
        self.node = node
        self.cabot_manager = cabot_manager
        self.subscribed_topics = set()
        self.start_time = time.time()
        self._timer = self.node.create_timer(5.0, self._check_temp_topics)

    def _check_temp_topics(self):
        elapsed = time.time() - self.start_time
        if elapsed > 180:
            common.logger.info('3 minutes passed, stopping temperature topic discovery.')
            self._timer.cancel()
            return

        topic_list = self.node.get_topic_names_and_types()
        for topic_name, types in topic_list:
            if 'sensor_msgs/msg/Temperature' in types and topic_name not in self.subscribed_topics:
                common.logger.info(f'Subscribing to {topic_name}')
                self.node.create_subscription(
                    sensor_msgs.msg.Temperature,
                    topic_name,
                    self._make_callback(topic_name),
                    10
                )
                self.subscribed_topics.add(topic_name)

    def _make_callback(self, topic_name):
        def callback(msg):
            self.cabot_manager.temperature_states(topic_name, msg)
        return callback


async def main():
    signal.signal(signal.SIGINT, sigint_handler)
    cabot_name = os.environ['CABOT_NAME'] if 'CABOT_NAME' in os.environ else None
    adapter_name = os.environ['CABOT_BLE_ADAPTER'] if 'CABOT_BLE_ADAPTER' in os.environ else "hci0"
    start_at_launch = (os.environ['CABOT_START_AT_LAUNCH'] == "1") if 'CABOT_START_AT_LAUNCH' in os.environ else False

    use_ble = os.environ.get('CABOT_USE_BLE', False)
    use_tcp = os.environ.get('CABOT_USE_TCP', True)
    common.logger.info(f"Use BLE = {use_ble}, Use TCP = {use_tcp}")

    cabot_manager = CaBotManager()
    cabot_manager.run(start=start_at_launch)

    if use_ble:
        result = subprocess.call(["grep", "-E", "^ControllerMode *= *le$", "/etc/bluetooth/main.conf"])
        if result != 0:
            common.logger.error("Please check your /etc/bluetooth/main.conf")
            line = subprocess.check_output(["grep", "-E", "ControllerMode", "/etc/bluetooth/main.conf"])
            common.logger.error("Your ControllerMode is '{}'".format(line.decode('utf-8').replace('\n', '')))
            common.logger.error("Please use ./setup_bluetooth_conf.sh to configure LE mode")
            sys.exit(result)

    global tcp_server
    global ble_manager
    global quit_flag
    global shutdown_client
    global set_24v_power_odrive_client

    def handleSpeak(req, res):
        res.result = False
        req_dictionary = message_to_ordereddict(req)
        req_dictionary['request_id'] = time.clock_gettime_ns(time.CLOCK_REALTIME)
        if ble_manager:
            ble_manager.handleSpeak(req_dictionary, res)
        if tcp_server:
            tcp_server.handleSpeak(req_dictionary, res)
        return res

    common.cabot_node_common.create_service(Speak, '/speak', handleSpeak)
    common.cabot_node_common.create_subscription(sensor_msgs.msg.BatteryState, '/battery_state', cabot_manager.battery_state, 10)
    common.cabot_node_common.create_subscription(power_controller_msgs.msg.BatteryArray, '/battery_states', cabot_manager.battery_states, 10)

    temperature_subscriber_manager = TemperatureSubscriberManager(common.cabot_node_common.sub_node, cabot_manager)

    shutdown_client = common.cabot_node_common.create_client(Trigger, "/shutdown")
    set_24v_power_odrive_client = common.cabot_node_common.create_client(SetBool, "/set_24v_power_odrive")

    global tcp_server_thread
    try:
        if use_tcp:
            if tcp_server is None:
                tcp_server = tcp.CaBotTCP(cabot_manager=cabot_manager)
                common.add_event_handler(tcp_server)
                if True:
                    tcp_server_thread = threading.Thread(target=tcp_server.start)
                    tcp_server_thread.start()
                else:
                    tcp_server.start()

        if use_ble:
            if ble_manager is not None:
                common.remove_event_handler(ble_manager)
            ble_manager = ble.BLEDeviceManager(adapter_name=adapter_name, cabot_name=cabot_name, cabot_manager=cabot_manager)
            common.add_event_handler(ble_manager)
            await ble_manager.run()
        else:
            while not quit_flag:
                time.sleep(1)
    except KeyboardInterrupt:
        common.logger.info("keyboard interrupt")
    except:  # noqa: E722
        common.logger.info(traceback.format_exc())
    cabot_manager.stop()
    common.logger.info("exiting the app")
    sys.exit(0)

if __name__ == "__main__":
    asyncio.run(main())
