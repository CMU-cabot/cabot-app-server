#!/usr/bin/python3

import os
import math
import logging
import rclpy
import rclpy.node
from std_srvs.srv import Empty, SetBool, Trigger
from sensor_msgs.msg import BatteryState
import subprocess
import sys
import threading
import traceback

from power_controller_ace.driver import BatteryDriver, BatteryStatus

DEBUG = False

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)


class BatteryDriverNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("power_controller")

        port_name = self.declare_parameter("port_name", os.environ.get('CABOT_ACE_BATTERY_PORT', 'dev/ttyACM0')).value
        baud = self.declare_parameter("baud", int(os.environ.get('CABOT_ACE_BATTERY_BAUD', '115200'))).value
        self.driver = BatteryDriver(port_name, baud)
        self.battery_thread = threading.Thread(target=self.driver.start)
        self.battery_thread.start()
        self.driver.delegate = self

        lowpower_shutdown_threshold = self.declare_parameter("lowpower_shutdown_threshold", 5).value
        self.driver.set_lowpower_shutdown_threshold(lowpower_shutdown_threshold)

        logger.info(f"{port_name=}, {baud=}, {lowpower_shutdown_threshold=}")

        self.connected = False
        self.systemctl_lock = threading.Lock()

        self.service0 = self.create_service(Empty, 'turn_jetson_switch_on', self.turn_jetson_switch_on)
        self.service1 = self.create_service(SetBool, 'set_12v_power', self.set_12v_power)
        self.service2 = self.create_service(SetBool, 'set_5v_power', self.set_5v_power)
        self.service3 = self.create_service(SetBool, 'set_odrive_power', self.set_odrive_power)
        self.shutdown_service = self.create_service(Trigger, 'shutdown', self.shutdown_callback)
        self.state_pub = self.create_publisher(BatteryState, "battery_state", 10)

        self.jetson_poweroff_commands = None
        jetson_user = self.declare_parameter("jetson_user", os.environ.get('CABOT_JETSON_USER', 'cabot')).value
        jetson_config = self.declare_parameter("jetson_config", os.environ.get('CABOT_JETSON_CONFIG', '')).value
        if jetson_config:
            id_file = self.declare_parameter("id_file", os.environ.get('CABOT_ID_FILE', 'id_ed25519')).value
            id_dir = self.declare_parameter("id_dir", os.environ.get('CABOT_ID_DIR', '~/.ssh')).value
            id_file_path = os.path.join(id_dir, id_file)
            if not os.path.exists(id_file_path):
                logger.error("ssh id file does not exist '{}'".format(id_file_path))
                sys.exit()

            self.jetson_poweroff_commands = []
            items = jetson_config.split()
            for item in items:
                split_item = item.split(':')
                if len(split_item) != 3:
                    logger.error("Invalid value of jetson_config(CABOT_JETSON_CONFIG) is found '{}'".format(item))
                    sys.exit()
                host = split_item[1]

                result = subprocess.call(["ssh", "-i", id_file_path, jetson_user + "@" + host, "exit"])
                if result != 0:
                    logger.error(F"Cannot connect Jetson host, please check ssh config. {jetson_user=}, {host=}, {id_file_path=}")
                    continue

                result = subprocess.call(["ssh", "-i", id_file_path, jetson_user + "@" + host, "sudo poweroff -w"])
                if result != 0:
                    logger.error(F"Cannot call poweroff on Jetson host, please check sudoer config. {jetson_user=}, {host=}, {id_file_path=}")
                    continue

                self.jetson_poweroff_commands.append(["ssh", "-i", id_file_path, jetson_user + "@" + host, "sudo poweroff"])
                logger.info(F"added shutdown command for {host}")

    def stop(self):
        self.driver.stop()
        self.battery_thread.join()

    def shutdown_callback(self, req, res):
        self.shutdown()
        res.success = True
        return res

    def shutdown(self):
        if self.jetson_poweroff_commands is not None:
            for command in self.jetson_poweroff_commands:
                logger.info("send shutdown request to jetson: %s", str(command))
                self._call(command, lock=self.systemctl_lock)
        self._call(["sudo", "systemctl", "poweroff"], lock=self.systemctl_lock)

    def _call(self, command, lock=None):
        result = 0
        if lock is not None and not lock.acquire(blocking=False):
            logger.info("lock could not be acquired")
            return result
        try:
            logger.info("calling %s", str(command))
            result = subprocess.call(command)
        except:  # noqa: E722
            logger.error(traceback.format_exc())
        finally:
            if lock is not None:
                lock.release()
        return result

    def battery_status(self, status: BatteryStatus):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "cabot-power-controller-ace"
        msg.percentage = float(status.battery_percentage / 100.0)
        msg.voltage = 36.0  # fixed value
        msg.current = math.nan  # Current not available
        msg.charge = math.nan  # Charge not available
        msg.capacity = math.nan  # Capacity not available
        msg.design_capacity = math.nan  # Design capacity not available
        msg.temperature = math.nan  # Temperature not available
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True

        self.state_pub.publish(msg)
        if status.shutdown:
            logger.info("shutdown requested")
            self.shutdown()
        if status.lowpower_shutdown:
            logger.info("lowpower shutdown requested")
            self.shutdown()

    def start(self):
        try:
            rclpy.spin(self)
        except:  # noqa: E722
            logger.error(traceback.format_exc())

    def turn_jetson_switch_on(self, req, res):
        self.driver.turn_jetson_switch_on()
        return res

    def set_12v_power(self, req, res):
        if req.data:
            self.driver.set_12v_power(1)
        else:
            self.driver.set_12v_power(0)
        res.success = True
        return res

    def set_5v_power(self, req, res):
        if req.data:
            self.driver.set_5v_power(1)
        else:
            self.driver.set_5v_power(0)
        res.success = True
        return res

    def set_odrive_power(self, req, res):
        if req.data:
            self.driver.set_odrive_power(1)
        else:
            self.driver.set_odrive_power(0)
        res.success = True
        return res


def main():
    rclpy.init()
    node = BatteryDriverNode()
    try:
        rclpy.spin(node)
    except:  # noqa: E722
        logger.error(traceback.format_exc())
    node.stop()


if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    main()
