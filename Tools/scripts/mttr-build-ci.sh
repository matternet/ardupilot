#!/bin/bash

set -ex

. ~/.profile

mkdir -p "$HOME/deploy_files"

python Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter
mv apm.pdef.xml "$HOME/deploy_files"

pushd ArduCopter
make -j8 px4-v2-mttr-m2
mv ArduCopter-v2.px4 "$HOME/deploy_files"
mv ../modules/PX4Firmware/Build/px4fmu-v2_APM.build/firmware.elf "$HOME/deploy_files/fmu.elf"
mv ../modules/PX4Firmware/Build/px4io-v2_default.build/firmware.elf "$HOME/deploy_files/io.elf"
make -j8 sitl
mv ArduCopter.elf "$HOME/deploy_files/sitl.elf"
popd
