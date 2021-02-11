#!/bin/bash

set -ex

rm -rf /tmp/deploy_files
mkdir -p "/tmp/deploy_files"

python Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter
mv apm.pdef.xml "/tmp/deploy_files"

unset CXX CC

wget https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2
tar xjf gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2
export PATH=$PATH:$PWD/gcc-arm-none-eabi-6-2017-q2-update/bin
echo $PATH

./waf distclean

./waf configure --board=MttrCubeBlack
./waf copter

./waf configure --board=MttrCubeOrange
./waf copter

./waf configure --board=sitl --debug
./waf copter

mv build/MttrCubeBlack/bin/arducopter.apj "/tmp/deploy_files/copter-MttrCubeBlack.apj"
mv build/MttrCubeOrange/bin/arducopter.apj "/tmp/deploy_files/copter-MttrCubeOrange.apj"
mv build/sitl/bin/arducopter "/tmp/deploy_files/sitl.elf"
