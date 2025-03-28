#!/bin/bash

stop_launch() {
    docker compose -f $dcfile --profile $profile down
    exit 0
}

trap 'stop_launch' SIGINT SIGTERM

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
prefix=$(basename `pwd`)

function help {
    echo "Usage: $0"
    echo ""
    echo "-h         show this help"
    echo "-d         development launch"
}

development=0
dcfile=docker-compose.yaml

while getopts "hd" arg; do
    case $arg in
    h)
        help
        exit
        ;;
    d)
        development=1
        ;;
    esac
done

log_prefix=cabot-app-server
log_name=${log_prefix}_`date +%Y-%m-%d-%H-%M-%S`

source $scriptdir/.env

if [ -n "$CABOT_LAUNCH_DEV_PROFILE" ]; then
    development=$CABOT_LAUNCH_DEV_PROFILE
fi

if [[ $(uname -a) =~ ^Darwin.*$ ]]; then
    profile=mac-prod
    if [[ $development -eq 1 ]]; then
        profile=mac-dev
        echo "This is development environment, building the workspace"
        com="docker compose -f $dcfile --profile $profile run --rm app-server-mac-dev /launch.sh build"
        echo $com
        eval $com
    fi

    com="docker compose -f $dcfile --profile $profile up 2>&1 | tee log/$log_name.log"
    echo $com
    eval $com
    exit
fi

profile=prod
if [[ $development -eq 1 ]]; then
    profile=dev
    echo "This is development environment, building the workspace"
    com="docker compose -f $dcfile --profile $profile run --rm app-server-dev /launch.sh build"
    echo $com
    eval $com
fi

com="docker compose -f $dcfile --profile $profile up 2>&1 | tee log/$log_name.log"
echo $com
eval $com
