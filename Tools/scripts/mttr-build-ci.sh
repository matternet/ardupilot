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

./waf configure --board=MttrCubeBlack --enable-opendroneid
./waf copter

./waf configure --board=MttrCubeOrange --enable-opendroneid
./waf copter

./waf configure --board=sitl --debug --enable-opendroneid
./waf copter

# Get Ardupilot version
eval $(sed -n 's/^#define  *\([^ ]*\)  *\(.*\) *$/export \1=\2/p' ArduCopter/version.h)

# Get mttr git tag (replace all dashes with underscores)
MTTR_GIT_TAG=$(echo $(git describe --tags --abbrev=0) | tr - _)

# Get mttr git hash
MTTR_GIT_HASH=$(git rev-parse --short=7 HEAD)

# Create version string
VERSION_DESCRIPTION="ap_${FW_MAJOR}.${FW_MINOR}.${FW_PATCH}-${MTTR_GIT_TAG}-${MTTR_GIT_HASH}"

mv build/MttrCubeBlack/bin/arducopter.apj "/tmp/deploy_files/copter-${VERSION_DESCRIPTION}-MttrCubeBlack.apj"
mv build/MttrCubeOrange/bin/arducopter.apj "/tmp/deploy_files/copter-${VERSION_DESCRIPTION}-MttrCubeOrange.apj"
mv build/MttrCubeBlack/bin/arducopter "/tmp/deploy_files/copter-${VERSION_DESCRIPTION}-MttrCubeBlack.elf"
mv build/MttrCubeOrange/bin/arducopter "/tmp/deploy_files/copter-${VERSION_DESCRIPTION}-MttrCubeOrange.elf"
mv build/sitl/bin/arducopter "/tmp/deploy_files/sitl.elf"
