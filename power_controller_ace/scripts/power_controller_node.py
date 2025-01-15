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
    def __init__(self, driver):
        super().__init__("battery_driver_node", start_parameter_services=False)
        self.driver = driver
        self.connected = False
        self.systemctl_lock = threading.Lock()

        self.service0 = self.create_service(Empty, 'turn_jetson_switch_on', self.turn_jetson_switch_on)
        self.service1 = self.create_service(SetBool, 'set_12v_power', self.set_12v_power)
        self.service2 = self.create_service(SetBool, 'set_5v_power', self.set_5v_power)
        self.service3 = self.create_service(SetBool, 'set_odrive_power', self.set_odrive_power)
        self.shutdown_service = self.create_service(Trigger, 'shutdown', self.shutdown_callback)
        self.state_pub = self.create_publisher(BatteryState, "battery_state", 10)
        self.declare_parameter("lowpower_shutdown_threshold", 10)
        self.add_on_set_parameters_callback(self.param_callback)
        self.jetson_poweroff_commands = None
        jetson_user = os.environ['CABOT_JETSON_USER'] if 'CABOT_JETSON_USER' in os.environ else "cabot"
        jetson_config = os.environ['CABOT_JETSON_CONFIG'] if 'CABOT_JETSON_CONFIG' in os.environ else None
        if jetson_config is not None:
            id_file = os.environ['CABOT_ID_FILE'] if 'CABOT_ID_FILE' in os.environ else ""
            id_dir = os.environ['CABOT_ID_DIR'] if 'CABOT_ID_DIR' in os.environ else ""
            id_file_path = os.path.join(id_dir, id_file)
            if not os.path.exists(id_file_path):
                logger.error("ssh id file does not exist '{}'".format(id_file_path))
                sys.exit()

            self.jetson_poweroff_commands = []
            items = jetson_config.split()
            for item in items:
                split_item = item.split(':')
                if len(split_item) != 3:
                    logger.error("Invalid value of CABOT_JETSON_CONFIG is found '{}'".format(item))
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

    def param_callback(self, params):
        for param in params:
            if param.name == "lowpower_shutdown_threshold":
                logger.info(f"{param.name}: {param.value}")
                self.driver.set_lowpower_shutdown_threshold(param.value)

    def battery_status(self, status: BatteryStatus):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "cabot2-ace-battery-control"
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

    port_name = os.environ['CABOT_ACE_BATTERY_PORT'] if 'CABOT_ACE_BATTERY_PORT' in os.environ else '/dev/ttyACM0'
    baud = int(os.environ['CABOT_ACE_BATTERY_BAUD']) if 'CABOT_ACE_BATTERY_BAUD' in os.environ else 115200
    driver = BatteryDriver(port_name, baud)
    battery_thread = threading.Thread(target=driver.start)
    battery_thread.start()
    node = BatteryDriverNode(driver)
    driver.delegate = node

    logger.info(f"{port_name=}, {baud=}")

    try:
        rclpy.spin(node)
    except:  # noqa: E722
        logger.error(traceback.format_exc())
    driver.stop()
    battery_thread.join()


if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    main()
