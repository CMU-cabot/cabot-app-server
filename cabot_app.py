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
import gzip
import math
import queue
import os
use_tcp = (os.environ['CABOT_APP_USE_TCP'] == "1") if 'CABOT_APP_USE_TCP' in os.environ else False
if use_tcp:
    import tcp
import time
import json
import threading
import traceback
import logging
import re
import signal
import subprocess
import sys
from uuid import UUID

import dgatt
import common
import ble

import roslibpy
from roslibpy.comm import RosBridgeClientFactory

from cabot import util
from cabot.event import BaseEvent
from cabot_ui.event import NavigationEvent
from cabot_ace import BatteryDriverNode, BatteryDriver, BatteryDriverDelegate, BatteryStatus

MTU_SIZE = 2**10 # could be 2**15, but 2**10 is enough
CHAR_WRITE_MAX_SIZE = 512 # should not be exceeded this value
#DISCOVERY_UUIDS=[str(CABOT_BLE_VERSION(0))]
DISCOVERY_UUIDS=[]
WAIT_AFTER_CONNECTION=0.25 # wait a bit after connection to avoid error



class DeviceStatus:
    def __init__(self):
        self.level = "Unknown"
        self.devices = []

    def ok(self):
        self.level = "OK"

    def error(self):
        self.level = "Error"

    def set_json(self, text):
        try:
            data=json.loads(text)
            self.devices = []
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
        except:
            common.logger.info(traceback.format_exc())

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

class CaBotManager(BatteryDriverDelegate):
    def __init__(self):
        self._device_status = DeviceStatus()
        self._cabot_system_status = SystemStatus()
        self._battery_status = None
        self.systemctl_lock = threading.Lock()
        self.start_flag = False
        self.stop_run = None
        self.check_interval = 1
        self.run_count = 0

    def run(self, start=False):
        self.start_flag=start
        self._run_once()
        self.stop_run = self._run()

    def stop(self):
        if self.stop_run:
            self.stop_run.set()

    # BatteryDriverDelegate start
    def battery_status(self, status):
        self._battery_status = status
        if status.shutdown or status.lowpower_shutdown:
            common.logger.info("shutdown requested")
            self.stop()
            self.poweroff()
    # BatteryDriverDelegate end

    @util.setInterval(5)
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
        if self._cabot_system_status.is_active():
            result = self._runprocess(["sudo", "-E", "./check_device_status.sh", "-j", "-s"])
        else:
            result = self._runprocess(["sudo", "-E", "./check_device_status.sh", "-j"])
        if result and result.returncode == 0:
            self._device_status.ok()
        else:
            self._device_status.error()
        self._device_status.set_json(result.stdout)

    def _check_service_active(self):
        result = self._runprocess(["systemctl", "--user", "is-active", "cabot"])
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

        #global diagnostics
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
            # common.logger.info("calling %s", str(command))
            result = subprocess.call(command)
        except:
            common.logger.error(traceback.format_exc())
        finally:
            if lock is not None:
                lock.release()
        return result

    def reboot(self):
        self._call(["sudo", "systemctl", "reboot"], lock=self.systemctl_lock)

    def poweroff(self):
        self._call(["sudo", "systemctl", "poweroff"], lock=self.systemctl_lock)

    def start(self):
        self._call(["systemctl", "--user", "start", "cabot"], lock=self.systemctl_lock)
        self._cabot_system_status.activating()

    def stop(self):
        self._call(["systemctl", "--user", "stop", "cabot"], lock=self.systemctl_lock)
        self._cabot_system_status.deactivating()

    def device_status(self):
        return self._device_status

    def cabot_system_status(self):
        return self._cabot_system_status

    def cabot_battery_status(self):
        return self._battery_status

quit_flag=False
ble_manager = None
tcp_server = None

def sigint_handler(sig, frame):
    common.logger.info("sigint_handler")
    global quit_flag
    global tcp_server
    global ble_manager
    if sig == signal.SIGINT:
        if ble_manager is not None:
            ble_manager.stop()
        quit_flag=True
        if tcp_server is not None:
            tcp_server.stop()
    else:
        common.logger.error("Unexpected signal")

def main():
    signal.signal(signal.SIGINT, sigint_handler)
    cabot_name = os.environ['CABOT_NAME'] if 'CABOT_NAME' in os.environ else None
    adapter_name = os.environ['CABOT_BLE_ADAPTER'] if 'CABOT_BLE_ADAPTER' in os.environ else "hci0"
    start_at_launch = (os.environ['CABOT_START_AT_LAUNCH'] == "1") if 'CABOT_START_AT_LAUNCH' in os.environ else False

    port_name = os.environ['CABOT_ACE_BATTERY_PORT'] if 'CABOT_ACE_BATTERY_PORT' in os.environ else None
    baud = int(os.environ['CABOT_ACE_BATTERY_BAUD']) if 'CABOT_ACE_BATTERY_BAUD' in os.environ else None
    
    global use_tcp

    cabot_manager = CaBotManager()
    cabot_manager.run(start=start_at_launch)

    driver = None
    if port_name is not None and baud is not None:
        driver = BatteryDriver(port_name, baud, delegate=cabot_manager)
        battery_driver_node = BatteryDriverNode(common.client, driver)
        battery_thread = threading.Thread(target=driver.start)
        battery_thread.start()

    result = subprocess.call(["grep", "-E", "^ControllerMode *= *le$", "/etc/bluetooth/main.conf"])
    if result != 0:
        common.logger.error("Please check your /etc/bluetooth/main.conf")
        line = subprocess.check_output(["grep", "-E", "ControllerMode", "/etc/bluetooth/main.conf"])
        common.logger.error("Your ControllerMode is '{}'".format(line.decode('utf-8').replace('\n', '')))
        common.logger.error("Please use ./setup_bluetooth_conf.sh to configure LE mode")
        sys.exit(result)
    
    def start_tcp():
        global tcp_server
        if tcp_server is None:
            tcp_server = tcp.CaBotTCP(cabot_manager=cabot_manager)
            if False:
                tcp_server_thread = threading.Thread(target=tcp_server.start)
                tcp_server_thread.start()
            else:
                common.logger.info("starting tcp server in main thread")
                tcp_server.start()
            common.event_handler = tcp_server
        else:
            common.logger.info("tcp_server is already running")

    global quit_flag
    global ble_manager
    global tcp_server
    tcp_server_thread = None
    common.logger.info("starting...")
    try:
        while not quit_flag:
            result = subprocess.call(["sudo", "rfkill", "unblock", "bluetooth"])
            if result != 0:
                common.logger.error("Could not unblock rfkill bluetooth")
                time.sleep(1)
                continue

            result = subprocess.call(["sudo", "systemctl", "restart", "bluetooth"])
            if result != 0:
                common.logger.error("Could not restart bluetooth service")
                time.sleep(1)
                continue
            if not use_tcp:
                ble_manager = ble.BLEDeviceManager(adapter_name=adapter_name, cabot_name=cabot_name, cabot_manager=cabot_manager)
                common.event_handler = ble_manager
                # power on the adapter
                try:
                    while not ble_manager.is_adapter_powered and not quit_flag:
                        common.logger.info("Bluetooth is off, so powering on")
                        ble_manager.is_adapter_powered = True
                        time.sleep(1)
                    ble_manager.start_discovery(DISCOVERY_UUIDS)
                    ble_manager.run()
                except KeyboardInterrupt:
                    common.logger.info("keyboard interrupt")
                    break
                except:
                    common.logger.info(traceback.format_exc())
                    time.sleep(1)
            else:
                start_tcp()
    except KeyboardInterrupt:
        common.logger.info("keyboard interrupt")
    except Exception as e:
        common.logger.info(traceback.format_exc())
    finally:
        try:
            if driver:
                driver.stop()
                battery_thread.join()
            if tcp_server:
                tcp_server.stop()
                if tcp_server_thread:
                    tcp_server_thread.join()
            if ble_manager:
                ble_manager.stop()
            common.client.terminate()
        except:
            common.logger.info(traceback.format_exc())

if __name__ == "__main__":
    main()
