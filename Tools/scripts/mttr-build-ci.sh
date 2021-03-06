#!/bin/bash

set -ex

rm -rf /tmp/deploy_files
mkdir -p "/tmp/deploy_files"

python Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter
mv apm.pdef.xml "/tmp/deploy_files"

wget --quiet https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2
tar xjf gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2
export PATH=$PATH:$PWD/gcc-arm-none-eabi-6-2017-q2-update/bin
echo $PATH

./waf configure --board=MttrCubeBlack
./waf copter

sudo apt-get install g++-7
export CXX=g++-7
export CC=gcc-7

./waf configure --board=sitl --debug
./waf copter

mv build/MttrCubeBlack/bin/arducopter.apj "/tmp/deploy_files/copter-MttrCubeBlack.apj"
mv build/sitl/bin/arducopter "/tmp/deploy_files/sitl.elf"
