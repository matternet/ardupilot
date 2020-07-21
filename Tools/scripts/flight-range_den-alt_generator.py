from __future__ import division
import pprint
import math
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum
import statistics

class plot(Enum):
        flight_range = 1
        watt_consumption = 2

def findslope(x1, x2, y1, y2):
        m = (float)(y2-y1)/(x2-x1)
        print "Slope is: ", m
        return m

def plot_line(x_arr, y_arr, title_str, xlabel, ylabel):
        fig, ax = plt.subplots()
        plt.plot(x_arr, y_arr, 'r-o', linewidth = 1.0, label='0 m/s (0 knots) head wind')
        ax.set_axisbelow(True) 
        ax.minorticks_on()

        # Customize the major grid
        ax.grid(which='major', linestyle='-', linewidth='0.5', color='red')
        # Customize the minor grid
        ax.grid(which='minor', linestyle=':', linewidth='0.5', color='black')

        plt.title(title_str)
        plt.xlabel(xlabel, color='#1C2833')
        plt.ylabel(ylabel, color='#1C2833')
        plt.grid(True)
        plt.legend()
        plt.show()

# plot density alt v/s vehicle range for a single graph
def plotRange_denAlt(denAlt, flightRanges):
        plt.plot(denAlt,flightRanges,c='r', linewidth = 3.0)
        plt.title('Graph of Density Alt v/s Vehicle Range')
        plt.xlabel('Density Altitude', color='#1C2833')
        plt.ylabel('Range', color='#1C2833')
        plt.grid()
        plt.show()

# Convert distance value from km to nautical miles
def km_to_nm(kms):
    nm = (float)(kms*0.539957)
    return nm

# Convert speed value from meter/sec to knots
def ms_to_knt(ms):
    knt = (float)(ms*1.94384)
    return knt

# Convert speed value from knots to meter/sec
def knt_to_ms(knt):
    ms = (float)(0.514444*knt)
    return ms

def m_to_ft(m):
        ft = (float)(3.28084*m)
        return ft

def ft_to_m(ft):
        m = (float)(0.3048*ft)
        return m

# ISA temperature (always 15 C at sea level)
def ISA(pressure_alt):
        return((-2*(pressure_alt/1000))+15)

# pressure altitude = (standard pressure - your current pressure setting) x 1,000 + field elevation
def pressure_alt(baro_pressure, msl):
        return (((29.92-baro_pressure)*1000)+msl)

# density altitude = pressure altitude + [120 x (OAT - ISA Temp)]
def densityAlt(pressure_alt, temp, isa_temp):
        return(pressure_alt+(120*(temp-isa_temp)))

def angle_of_attack(velocity, wind_ms):
        max_pitch_angle = 20
        if (wind_ms+velocity <= max_pitch_angle):       # The effect of head wind on the range is minimal if the vehicle can still attain its max tilt angle which is 20 degrees
                return max_pitch_angle
        else:
                return max_pitch_angle - ((wind_ms+velocity)-max_pitch_angle)
        
# convert temperature values from celcius to kelvin
def cel_to_kel(celcius):
        return (float)(celcius+273.15)

#convert pressure values from inHg to Pascal
def inHg_to_pascal(inHg):
        return (float)(inHg*3386.39)

#convert pressure values from Pascal to inHg
def pascal_to_inHg(pascal):
        return (float)(pascal*0.0002953)

#Finds air density based on the msl values
def find_airDensity(msl): #ad = 
        if (msl==0):
                return 1.22

        if (msl>0 and msl<=1000):
                return 1.15

        if (msl>1000 and msl<=2000):
                return 1.12

        if (msl>2000 and msl<=3000):
                return 1.07

        if (msl>3000 and msl<=4000):
                return 1.04

        if (msl>4000 and msl<=5000):
                return 0.99

        if (msl>5000 and msl<=6000):
                return 0.96

        if (msl>6000 and msl<=7000):
                return 0.93

        if (msl>7000 and msl<=8000):
                return 0.90

        if (msl>8000 and msl<=9000):
                return 0.86

        if (msl>9000 and msl<=10000):
                return 0.83

#Finds Dynamic air viscosity in kg/ms based on the temp values (in C)
def find_airViscosity(temp): # (in kg/ms)
        if (temp < 0 and temp >= -10):
                return 1.680*(10**(-5))

        if (temp==0):
                return 1.729*(10**(-5))

        if (temp>0 and temp<=5):
                return 1.754*(10**(-5))

        if (temp>5 and temp<=10):
                return 1.778*(10**(-5))

        if (temp>10 and temp<=15):
                return 1.802*(10**(-5))

        if (temp>15 and temp<=20):
                return 1.825*(10**(-5))

        if (temp>20 and temp<=25):
                return 1.849*(10**(-5))

        if (temp>25 and temp<=30):
                return 1.872*(10**(-5))

        if (temp>30 and temp<=35):
                return 1.895*(10**(-5))

        if (temp>35 and temp<=40):
                return 1.918*(10**(-5))

        if (temp>40 and temp<=45):
                return 1.941*(10**(-5))

def find_wind_vel_coeff(wind_ms, velocity):
        max_pitch_angle = 20

        if (wind_ms+velocity <= max_pitch_angle):       # The effect of head wind on the range is minimal if the vehicle can still attain its max tilt angle which is 20 degrees
            wind_vel_coeff = 1-(wind_ms*0.028)          # Takes 2.8% of the flight range for each wind_speed unit

        else:                                           # wind_vel_coeff = 1-(((wind_ms+velocity)-20)/(velocity))
            velocity = velocity - ((wind_ms+velocity)-max_pitch_angle)
            wind_vel_coeff = 1-(wind_ms*0.032)          # Takes ~3.2% of the flight range for each wind_speed unit
#            wind_vel_coeff = 1-(wind_ms/(2*velocity))
#            wind_vel_coeff = 1-((float)(wind_ms/2)/(velocity))
        return wind_vel_coeff

def watt_after_losses(velocity, watt, wind_speed):
        wind_speed_ms = knt_to_ms(wind_speed)
        wind_vel_coeff = 1 + (1-find_wind_vel_coeff(wind_speed_ms, velocity))
        final_watt = watt*wind_vel_coeff
#        print "\n\n\nwind_vel_coeff is --------- ", wind_vel_coeff, "\n\n\n"
        return final_watt

def watt_after_losses_array(watt_arr, velocity, wind_speed):
        final_watt_arr = []
        for watt in watt_arr:
                final_watt_arr.append(watt_after_losses(velocity, watt, wind_speed))
        return final_watt_arr

def flightRange(velocity, watt, battery_capacity):
        AAD = (11.2*(watt/10))/50.4                             # calculate avg amp draw
        flightTime = ((battery_capacity*0.8)/AAD)*60            # calculate total flight time for given battery capacity and avg amp draw
        fRange = (float)(((flightTime-2)*velocity*60)/1000)     # calculate flight range given the velocity and flight time
        return fRange                                           # return flight range

# Give out the flight range array for an array of wattage used
def flight_range_array(vel, wind_vel, bat_cap, watt_arr):
        flightRanges = []
        print vel, "m/s +", wind_speed, " m/s Head wind"
        for watt in watt_arr:                                                   # Go through the wattage array and get flight range for each wattag consumption value
                flightRanges.append(km_to_nm(flightRange(vel, watt, bat_cap)))  # and insert that into a list of flight range for that corresponding wattage consumption value
#        pprint.pprint(flightRanges)
        return flightRanges                                                     # return the flight ranges array (corresponding to the wattage consumption array values)

def graph_multiple_plots(vehicle_vel, wind_speed_range, watt_arr, battery_capacity, da_array, graph_type, mass):
        y_axis_arr = []
        
        if (graph_type == plot.flight_range):
                label_string = "Graph of Density Alt v/s Vehicle range - [Configured enroute cruise speed for the aircraft:" + str(round(ms_to_knt(vehicle_vel), 2)) + " knots (" + str(vehicle_vel) + " m/s), " + str(mass-11.2) + "Kg Payload]"
                title_str = str(label_string)
                xlabel = 'Density Altitude (Ft)'
                ylabel = 'Range (Nautical miles) - [20% battery reserves taken into consideration]'

        elif (graph_type == plot.watt_consumption):
                label_string = "Graph of Density Alt v/s Wattage consumption - [Configured enroute cruise speed for the aircraft:" + str(round(ms_to_knt(vehicle_vel), 2)) + " knots (" + str(vehicle_vel) + " m/s), " + str(mass-11.2) + "Kg Payload]"
                title_str = str(label_string)
                xlabel = 'Density Altitude (Ft)'
                ylabel = 'Wattage - [20% battery reserves taken into consideration]'                

        fig, ax = plt.subplots()

        for head_wind_vel_knt in wind_speed_range: #head wind in knots
#                head_wind_vel_knt = ms_to_knt(head_wind_vel)
                final_watt_array = watt_after_losses_array(watt_arr, vehicle_vel, head_wind_vel_knt)

                if (graph_type == plot.flight_range):
                        final_range_nm_array = flight_range_array(vehicle_vel, head_wind_vel_knt, battery_capacity, final_watt_array)
                        y_axis_arr = final_range_nm_array
                elif (graph_type == plot.watt_consumption):
                        y_axis_arr = final_watt_array

                if (head_wind_vel_knt == 0):
                        plt.plot(da_array, y_axis_arr, 'r', linewidth = 3.0, label='0 knt (0 m/s) head wind')
                
                if (head_wind_vel_knt == 1):
                        plt.plot(da_array, y_axis_arr, 'g', linewidth = 3.0, label='1 knt (0.51 m/s) head wind')

                if (head_wind_vel_knt == 2):
                        plt.plot(da_array, y_axis_arr, 'b', linewidth = 3.0, label='2 knt (1.02 m/s) head wind')

                if (head_wind_vel_knt == 3):
                        plt.plot(da_array, y_axis_arr, 'c', linewidth = 3.0, label='3 knt (1.54 m/s) head wind')

                if (head_wind_vel_knt == 4):
                        plt.plot(da_array, y_axis_arr, '0.75', linewidth = 3.0, label='4 knt (2.05 m/s) head wind')

                if (head_wind_vel_knt == 5):
                        plt.plot(da_array, y_axis_arr, 'y', linewidth = 3.0, label='5 knt (2.57 m/s) head wind')

                if (head_wind_vel_knt == 6):
                        plt.plot(da_array, y_axis_arr, 'k', linewidth = 3.0, label='6 knt (3.08 m/s) head wind')

                if (head_wind_vel_knt == 7):
                        plt.plot(da_array, y_axis_arr, '', linewidth = 3.0, label='7 knt (3.60 m/s) head wind')

                if (head_wind_vel_knt == 8):
                        plt.plot(da_array, y_axis_arr, 'lightcoral', linewidth = 3.0, label='8 knt (4.11 m/s) head wind')
                
                if (head_wind_vel_knt == 9):
                        plt.plot(da_array, y_axis_arr, 'r', linewidth = 3.0, label='9 knt (4.63 m/s) head wind')
                
                if (head_wind_vel_knt == 10):
                        plt.plot(da_array, y_axis_arr, 'g', linewidth = 3.0, label='10 knt (5.14 m/s) head wind')

                if (head_wind_vel_knt == 11):
                        plt.plot(da_array, y_axis_arr, 'b', linewidth = 3.0, label='11 knt (5.65 m/s) head wind')

                if (head_wind_vel_knt == 12):
                        plt.plot(da_array, y_axis_arr, 'c', linewidth = 3.0, label='12 knt (6.17 m/s) head wind')

                if (head_wind_vel_knt == 13):
                        plt.plot(da_array, y_axis_arr, '0.75', linewidth = 3.0, label='13 knt (6.68 m/s) head wind')

                if (head_wind_vel_knt == 14):
                        plt.plot(da_array, y_axis_arr, 'y', linewidth = 3.0, label='14 knt (7.20 m/s) head wind')

                if (head_wind_vel_knt == 15):
                        plt.plot(da_array, y_axis_arr, 'k', linewidth = 3.0, label='15 knt (7.71 m/s) head wind')

                if (head_wind_vel_knt == 16):
                        plt.plot(da_array, y_axis_arr, '', linewidth = 3.0, label='16 knt (8.23 m/s) head wind')


#       FOr seeing the data point itself highlighted                 plt.plot(da_array, y_axis_arr, 'm-o', linewidth = 1.0, label='7 m/s (13.6 knots) head wind')

#                for x,y in zip(da_arr, y_axis_arr):
#                        print "Density Alt: ", x, ", W/Range: ", y
                y_axis_arr = []

        ax.set_axisbelow(True)
        ax.minorticks_on()

        # Customize the major grid
        ax.grid(which='major', linestyle='-', linewidth='0.5', color='red')
        # Customize the minor grid
        ax.grid(which='minor', linestyle=':', linewidth='0.5', color='black')

        plt.title(title_str, fontweight='bold')
        plt.xlabel(xlabel, color='#1C2833', fontsize='large')
        plt.ylabel(ylabel, color='#1C2833', fontsize='large')
        plt.grid(True)
        plt.legend()
        plt.show()


def get_air_density(baro_pressure, gas_constant, temperature):
        air_density = (float)(inHg_to_pascal(baro_pressure)/(gas_constant*cel_to_kel(temperature))) #Air density (kg/m^3), Pressure (Pascal), R = Gas constant(J/(kg*degK)) - 287.05 for dry air, T = temp(Kelvin)-C+273.15
        return air_density

def get_watt_consumption(drag_coefficient, crossectional_area, vehicle_vel, wind_speed, air_density, mass):

        drag = 0.5*(air_density*((vehicle_vel-wind_speed)**2)*drag_coefficient*crossectional_area) # drag coefficient  = zero-lift drag + lift induced drag. The higher the speed the higher the drag

        mass = 13.2 # in kg
        Thrust = (((mass*9.8)**2)+(drag**2))**0.5 #sqrt((tx^2) + (ty^2)), where tx = T.sin(O) & ty = T.cos(O), and [T.sin(O)=m.g], [T.cos(O)=drag force] 

        print "DRAG IS: ", drag, "| THRUST IS: ", Thrust, "| cos of angle of attack IS: ", math.cos(math.radians(angle_of_attack(vehicle_vel, wind_speed)))  

        # calculate power(in Watt) consumed based on the drag and and density alt (air_density)
        W = (float)(Thrust*(((Thrust/Rotor_Area)*(1/(2*air_density)))**0.5))

        #account for real world power losses before getting final Wattage value, not accounting for wind_effect atm.
        W = (float)(W*real_world_watt_coeff)

        return W


def get_watt_array(drag_coefficient, crossectional_area, vehicle_vel, wind_speed, air_density):
        watt = get_watt_consumption(thrust, drag_coefficient, crossectional_area, vehicle_vel, wind_speed, air_density)
        watt_arr.append(watt)


#-----------------Main-------------------------

# Make Density Alt Array from 0 ft to 10,000 ft
da_start = 0 #DA in ft
da_end =  10000 #DA in ft
da_interval = 10 # in ft
da_arr = np.arange(da_start, da_end, da_interval)

# Constants
Rotor_Area = 0.98       #in m^2 (22 inch prop * 7.8 inch pitch)
real_world_watt_coeff = 1.45 # in W, Defines for power losses in real world - 45% increase in wattage use
drag_coefficient = 1.05 # 1,05 is for a cube, 1.28 is for a flat plate 
gas_constant = 287.05 # in J, for dry air
crossectional_area = 0.07 # in m^2 (10 inch * 11 inch)
battery_capacity = 13 # Ah (20% kept as reserve)

# Variables
vehicle_vel = 16 # m/s
mass = 13.2 # (in Kg) Vehicle default is taken as 11.2 kg, 2 kg for payload
wind_speed_range = range(16) # in knots
wind_speed = 0 # in m/s

MSL = list(range(0,10500,500))
TEMP = list(range(-4,40,4))
baro_pressure = [29.84, 30.04] #in Inches Hg (#29.84 for MSL Inches Hg, got from: https://aviationweather.gov/adds/metars/index?chk_metars=on&chk_tafs=on&station_ids=KHAF&std_trans=translated)

watt_arr = []
final_den_alt_points_arr = []
den_alt_W_2d_list = []

old_den_alt = 0
old_air_den = 0
old_watt = 0
for temp in TEMP:
        for msl in MSL:
                for bp in baro_pressure:
                        
                        # get air density
                        air_density = get_air_density(bp, gas_constant, temp) 
#                        air_density = find_airDensity(msl)

                        # get density alt
                        pa = pressure_alt(bp, msl)
                        isa = ISA(pa)
                        den_alt = densityAlt(pa, temp, isa)

                        #get Watt consumption for these parameters (msl, temp, bp, denAlt)
                        watt_val = get_watt_consumption(drag_coefficient, crossectional_area, vehicle_vel, wind_speed, air_density, mass)

                        # append density alt, watt in the final array (array of 'x' & 'y' coordinates respectively)

                        if (den_alt >= 0 and air_density!=old_air_den):
#                                final_den_alt_points_arr.append(den_alt)
#                                watt_arr.append(watt_val)
                                print "watt val - ", watt_val, "> old watt: ", old_watt
                                print "\nMSL: ", msl, "|\ttemp: ", temp, "|\tbp: ", bp, "|\tair_density: ", air_density, "|\tden_alt: ", den_alt, "|\tWattage: ", watt_val

                                den_alt_W_2d_list.append([den_alt, watt_val])
                        
                        old_watt = watt_val
                        old_den_alt = den_alt
                        old_air_den = air_density
                        


den_alt_W_2d_list.sort(key=lambda x: x[0])

curve_fit_da = []
curve_fit_w = []
curve_fit_2d = []

count = 0
avg_i=0
avg_j=0
n = 20
avg_i_list = []
avg_j_list = []

for i in den_alt_W_2d_list:
        if (count<n):
                avg_i = avg_i+i[0]
                avg_j = avg_j+i[1]
                avg_i_list.append(i[0])
                avg_j_list.append(i[1])
                count += 1
        else:
                avg_i = avg_i/n
                avg_j = avg_j/n
#                avg_i = statistics.median(avg_i_list) # Median
#                avg_j = statistics.median(avg_j_list) # Median
#                curve_fit_da.append(avg_i)
#                curve_fit_w.append(avg_j)
                curve_fit_2d.append([avg_i, avg_j])
                avg_i = 0
                avg_j = 0
                avg_i_list = []
                avg_j_list = []
                count = 0
        
for i in den_alt_W_2d_list:
        print "Density Alt: ", i[0], ", Watt: ", i[1]


#for x,y in zip(curve_fit_da, curve_fit_w):
#        print "Density Alt: ", x, ", Watt: ", y

print "size of curve 2d: ", len(curve_fit_2d)

# Make a new array just to get the plot function working
new_final_den_alt_points_arr = []
new_watt_arr = []
for i in curve_fit_2d:
#        print i
        new_final_den_alt_points_arr.append(i[0])
        new_watt_arr.append(i[1])        

# Wattage plot - 2kg payload, 16 ms speed
graph_multiple_plots(vehicle_vel, wind_speed_range, new_watt_arr, battery_capacity, new_final_den_alt_points_arr, plot.watt_consumption, mass)

# Range plot - 2kg payload, 16 ms speed
graph_multiple_plots(vehicle_vel, wind_speed_range, new_watt_arr, battery_capacity, new_final_den_alt_points_arr, plot.flight_range, mass)
