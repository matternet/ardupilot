#!/bin/bash

set -ex

. ~/.profile

git submodule update --recursive

mkdir -p "$HOME/deploy_files"

python Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter
mv apm.pdef.xml "$HOME/deploy_files"

"./modules/waf/waf-light" distclean
"./modules/waf/waf-light" configure --board px4-v2
"./modules/waf/waf-light" -j8 --targets bin/arducopter-quad
mv build/px4-v2/bin/arducopter-quad "$HOME/deploy_files/fmu.elf"
mv build/px4-v2/bin/arducopter-quad.px4 "$HOME/deploy_files/arducopter-quad.px4"

"./modules/waf/waf-light" distclean
"./modules/waf/waf-light" configure --board sitl
"./modules/waf/waf-light" -j8 --targets bin/arducopter-quad
mv build/sitl/bin/arducopter-quad "$HOME/deploy_files/sitl.elf"

"./modules/waf/waf-light" distclean
CXX=arm-linux-gnueabihf-g++  CC=arm-linux-gnueabihf-gcc "./modules/waf/waf-light" configure --board sitl
"./modules/waf/waf-light" -j8 --targets bin/arducopter-quad
mv build/sitl/bin/arducopter-quad "$HOME/deploy_files/sitl-armhf.elf"
