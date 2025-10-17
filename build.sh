#!/bin/sh
. ~/.bashrc
colcon build --symlink-install
. install/setup.sh