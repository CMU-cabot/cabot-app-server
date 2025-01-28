#!/bin/bash

if [ $(id -u) -eq 0 ]; then
   echo "please do not run as root: $0"
   exit
fi

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

## install cabot-ble-server
sudo ln -sf $scriptdir /opt/cabot-ble-server
docker pull cmucal/cabot-app-server

## install cabot-ble-server.service
INSTALL_DIR=$HOME/.config/systemd/user
mkdir -p $INSTALL_DIR
cp $scriptdir/cabot-ble-server.service $INSTALL_DIR
systemctl --user daemon-reload
systemctl --user enable cabot-ble-server
