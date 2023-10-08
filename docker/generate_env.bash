#!/usr/bin/bash

# get parameter from system
user=`id -un`
group=`id -gn`
uid=`id -u`
gid=`id -g`

echo "USER=${user}" > .env
echo "GROUP=${group}" >> .env
echo "UID=${uid}" >> .env
echo "GID=${gid}" >> .env
echo "WORKSPACE_DIR=${HOME}/work" >> .env
echo "SERIAL_DEVICE=/dev/ttyACM0" >> .env
