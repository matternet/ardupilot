#!/usr/bin/env python

import pexpect, time, sys
from pymavlink import mavutil

def wait_heartbeat(mav, timeout=10):
    '''wait for a heartbeat'''
    start_time = time.time()
    while time.time() < start_time+timeout:
        if mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2) is not None:
            return
    print("Failed to get heartbeat")    

def wait_mode(mav, modes, timeout=10):
    '''wait for one of a set of flight modes'''
    start_time = time.time()
    last_mode = None
    while time.time() < start_time+timeout:
        wait_heartbeat(mav, timeout=10)
        if mav.flightmode != last_mode:
            print("Flightmode %s" % mav.flightmode)
            last_mode = mav.flightmode
        if mav.flightmode in modes:
            return
    print("Failed to get mode from %s" % modes)
    sys.exit(1)

def wait_time(mav, simtime):
    '''wait for simulation time to pass'''
    imu = mav.recv_match(type='RAW_IMU', blocking=True)
    t1 = imu.time_usec*1.0e-6
    while True:
        imu = mav.recv_match(type='RAW_IMU', blocking=True)
        t2 = imu.time_usec*1.0e-6
        if t2 - t1 > simtime:
            break

cmd = '../Tools/autotest/sim_vehicle.py -D -S10'
mavproxy = pexpect.spawn(cmd, logfile=sys.stdout, timeout=30)
time.sleep(3)

mav = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

mavproxy.expect("using GPS")
mavproxy.expect("using GPS")
mavproxy.send('mode GUIDED\n')
wait_mode(mav, ['GUIDED'])
mavproxy.send('param set SIM_MAG_OFS_X 0\n')
mavproxy.send('arm throttle\n')
mavproxy.expect("Arming motors")
mavproxy.send('long MISSION_START\n')
#mavproxy.expect("Mission: 6 Jump")
#mavproxy.send('param set SIM_MAG1_FAIL 1\n')
mavproxy.expect("Disarming motors")

#mavproxy.send('param set SIM_MAG1_FAIL 1\n')
#mavproxy.expect("switching to compass")
#mavproxy.expect("switching to compass")

mavproxy.send('param set SIM_MAG_OFS_X 100\n')
mavproxy.send('mode GUIDED\n')
wait_mode(mav, ['GUIDED'])
mavproxy.send('arm throttle force\n')
mavproxy.expect("Arming motors")
mavproxy.send('long MISSION_START\n')
mavproxy.expect("Reached command #5")
mavproxy.send("param set SIM_MAG_FAIL_MSK 1\n")
#mavproxy.send('module load console\n')
#mavproxy.send('module load map\n')
#mavproxy.send('map set showsimpos 1\n')
#mavproxy.logfile = None
#mavproxy.interact()
mavproxy.expect("Disarming motors")
mavproxy.logfile = None
