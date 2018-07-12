#!/bin/bash

set -ex

. ~/.profile

mkdir -p "$HOME/deploy_files"

python Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter
mv apm.pdef.xml "$HOME/deploy_files"

unset CXX CC

./waf distclean
./waf configure --board=fmuv3
./waf build --targets bin/arducopter -j8
./waf configure --board=sitl
./waf build --targets bin/arducopter -j8

mv build/fmuv3/bin/arducopter.apj "$HOME/deploy_files"
mv build/fmuv3/bin/arducopter "$HOME/deploy_files/fmu.elf"
mv build/sitl/bin/arducopter "$HOME/deploy_files/sitl.elf"
