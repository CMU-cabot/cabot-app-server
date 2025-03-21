# CaBot BLE Server

BLE server to monitor/control [CaBot](https://github.com/cmu-cabot/cabot)

# Install

- edit `.env` file

```
git submodule update --init --recursive
./install.sh
```

# Uninstall

```
./uninstall.sh
```


# Environment Variables (see .env file as example)
```
CABOT_NAME                  # cabot name
CABOT_BLE_ADAPTER           # default is 'hci0'
CABOT_START_AT_LAUNCH       # default is 0, launch cabot system if 1 at start up
CABOT_USE_BLE               # default is false, use BLE if true
CABOT_USE_TCP               # default is true,  use TCP if true
CABOT_LAUNCH_DEV_PROFILE    # default is 0, launch in development mode if 1
HOST_UID                    # host user UID (default=1000)
HOST_GID                    # host user GID (default=1000)
HOST_TZ                     # host timezone (default=UTC)
```

## variables for cabot2-ace battery configuration
```
CABOT_ACE_BATTERY_PORT=/dev/ttyESP32
CABOT_ACE_BATTERY_BAUD=115200
```

## variables for device check
```
LIDAR_IF=enp8s0
LIDAR_IP=192.168.2.201
MICRO_CONTROLLER=Arduino
ARPSCAN_LIDAR		# prefix lidar name(default is Velodyne).If you use Hesai Lidar, set this to Hesai.
```

### related to model variant
- GPU PC + Realsense x 3
```
CABOT_REALSENSE_SERIAL_1
CABOT_REALSENSE_SERIAL_2
CABOT_REALSENSE_SERIAL_3
CABOT_CAMERA_NAME_1
CABOT_CAMERA_NAME_2
CABOT_CAMERA_NAME_3
```
- NUC + Jetson (Realsense x 3)
```
# User name to login jetson (default=cabot)
CABOT_JETSON_USER=cabot
# specify jetson mate config. expects a config which is provided for cabot
# jetson hosts will be automaticallly shutdown in the written order
CABOT_JETSON_CONFIG="D:192.168.1.52:rs3 D:192.168.1.51:rs2 D:192.168.1.50:rs1"
# ssh id file to log in jetson
CABOT_ID_FILE=id_ed25519_cabot
```

On Jetson hosts, please use "visudo" command to modify "/etc/sudoer" and add the following line to shutdown Jetson automatically.
```
cabot   ALL=NOPASSWD: /sbin/poweroff
```

##


# Test

- if you pull the latest docker images from docker hub, run the following command

```
docker compose --profile build pull
```

- if you build docker image on your environment, run the script to build image

```
./bake-docker.sh -i              # run docker image build
```

- launch ble server
```
./launch.sh     # launch in production mode
./launch.sh -d  # launch in development mode
```

- launch cabot_ace battery driver alone
```
CABOT_ACE_BATTERY_PORT=/dev/ttyXXXX CABOT_ACE_BATTERY_BAUD=115200 python3 -m cabot_ace.cabot_ace_battery_driver

options
-od,　　--odrive_power    #　turn on[1] or off[0] odrive motor controller.
```

