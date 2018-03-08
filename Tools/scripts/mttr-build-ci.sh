#!/bin/bash

set -ex

. ~/.profile

mkdir -p "$HOME/deploy_files"

python Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter
mv apm.pdef.xml "$HOME/deploy_files"

# export PX4_PX4IO_NAME="px4io-v2"

unset CXX CC

./waf distclean
./waf configure --board=px4-v3
./waf build --targets bin/arducopter -j8
./waf configure --board=sitl
./waf build --targets bin/arducopter -j8

mv build/px4-v3/bin/arducopter.px4 "$HOME/deploy_files"
mv build/px4-v3/bin/arducopter "$HOME/deploy_files/fmu.elf"
mv build/px4-v3/px4-extra-files/px4io "$HOME/deploy_files/io.elf"
mv build/sitl/bin/arducopter "$HOME/deploy_files/sitl.elf"
