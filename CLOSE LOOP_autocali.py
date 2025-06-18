####################################################################################
########### WELCOME! ###############################################################
# My name is FrED and I'm the brains of this operation,
# Congratulations on assembling the FrED device and setting up your remote desktop connection. 
# I'll be your instructor guiding you along your learning experience, 
# let's have some fun!
#
# Get ready to learn about:
# 1) Coding in Python 
# 2) Using controls to improve diameter control of fiber optic cable 
# 3) Varying extruder speed, spooling speed, and temperature to grow fiber optic cables
#    of desired diameter
# 
############ READ ME BEFORE STARTING, THIS IS THE FIRST PART ####################### 
# If you've opened this code, I'll assume you're ready for your first test!
# You can run this code using the green "Run" button in the bar above.
#
# The live video feed should come up of both the binary and live video, 
# nothing will show until a fiber is in front of the camera. 
# Your first run should start the spooling motor first, then the black block at the top
# labeled "HOT" will start heating up. 
# Once heated to the desired temperature (first run will be 95 C, be careful!)
# the extruder motor will start running. 
# 
# Your first task is to insert the hot glue preform, careful as the glue will be hot as it comes out. 
# 
# When the glue starts falling as a continuous thread, you can loop it around the 
# tensioners and attach it to the spool. 
# Be patient as this sometimes takes a few tries to get it right :)
#
# At this point you can stop the code and adjust the hardware location, 
# cooling fan position, and lighting conditions as necessary 
# until you get a continuous spooling and a stable image. 
# Make sure you take some time setting up this first run, everything else will be 
# much more fun with a clear image and stable fiber growth!
# 
##################### DO NOT CONTINUE UNTIL YOU DO THE FIRST PART #################
# Congratulations on getting your first fiber to spool! Your image looks great, 
# and the binary image matches the live feed such that a stable diameter is being detected.
# Let's keep going to the fun part! 
#
############### PYTHON AND CONTROLS INTERFACE (OPTIONAL READ)######################
# If you've written code in python, work in fiber optics (or have familiarity), or 
# have a controls background you're already halfway there to getting the most out of me!
# If you don't then you're in the right place to learn.
# 
# The code below is designed to walk you through step by step how the code is operating,
# directing you to the places where changes can be made to improve the quality of the fiber.
# Remember that I am a learning device, don't be afraid to add comments or ask questions!
#
# BRIEF DESCRIPTION OF INTERFACES:
# Running the program: The user interface that pops up after pressing the 
#                      start button is designed to represent how the code is setup and 
#                      allow the user to make controls and diameter changes without 
#                      knowing python or altering the code itself.
#
# This Code:           The code below is designed to walk you through step by step 
#                      how the code is operating, directing you to the places where 
#                      changes can be made to improve the quality of the fiber.
#                      Remember that I am a learning device, don't be afraid to add 
#                      more code or play around with different conditions to see their 
#                      effect on the fiber! Best coding practices for major edits is to 
#                      copy this document and rename it with something unique that you'll 
#                      remember.
# 
# Physical FrED:       The goal of the Fiber optic Extrusion Device is to spool hot glue from a larger 
#                      diameter to a smaller one and achieve precise diameter control. 
#                      This push-pull method is used in industry, usually using glass,
#                      to make fiber optic cables like those used in high speed internet applications.
#                      This diamter control is achieved by melting the hot 
#                      glue preform rod - not so hot that it liquifies, and not so cold that it can't 
#                      be spooled into thread. Then the extrude motor and spooling motor speeds are 
#                      adjusted so that volumetric flow rate is conserved (think volume in the larger cylinder,  
#                      or preform, input must match the volume in the thin cylinder, or fiber, coming out).

############################################################################################
############### BEGGINING OF USER CHANGEABLE CODE ##########################################
# In this section you'll find the code that allows you to change the initial hardware settings
# This includes rpm of the extruder motor, duty cycle of the spooling motor, and the temperature of the heater.

# ~~~~~~~ Extruder (Stepper) Motor Settings ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
direction = 0 # Clockwise = 1 or Anticlockwise = 0
microstepping = '1/2'      # Enter '1' , '1/2', '1/4', '1/8', '1/16' or '1/32', with colons
rpm = 0.3            #revolutions per minute RPM

# ~~~~~~~ Heater Settings ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
targetTemp = 80       #degree celcius

# ~~~~~~~ Spooling (DC) Motor Settings ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dcDuty = 10 #DC motor initialization  
                                                                                                                                                                                                                                                                                                                                                           

# ~~~~~~~ Fan Settings ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
fanDuty = 45 #duty cycle from 0 to 100%

# ~~~~~~~ Image Settings ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
binary_true = 1 # If binary_true = 1, use binary image. If 0, use gray image

##################### END OF USER CHANGEABLE CODE ##########################################
############################################################################################
# ONLY CHANGE CODE IF YOU ARE FAMILIAR WITH PYTHON AND HAVE AN UNDERSTANDING OF THE FUNCTIONS!
# Feel free to read along in the comments to see how the code works, change at your own risk
# To "play" with the code, try saving a copy of this document with a unique name and play around!
# However, you'll see that most of the controls interfaces can be accessed from the slider bars  
# in the user interface.

# ~~~~~~ Import Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import os
import cv2
import csv
import time
import math
import board
import busio
import atexit
import datetime
import digitalio
import contextlib
import threading
import numpy as np
import RPi.GPIO as GPIO
import adafruit_mcp3xxx.mcp3008 as MCP
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
from statistics import mean
from statistics import stdev
import tkinter as tk
from tkinter import ttk
GPIO.setmode(GPIO.BCM)

from time import sleep
from gpiozero import Motor
from gpiozero import RotaryEncoder
from adafruit_mcp3xxx.analog_in import AnalogIn
import warnings
warnings.filterwarnings("ignore")
import sys
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QSlider, QGridLayout, QWidget, QDoubleSpinBox, QPushButton, QMessageBox, QLineEdit, QCheckBox
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from PyQt5.QtGui import QPixmap

#~~~~~~~~~~~~~~~~~~~~ Initialization ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#Parameters for motor calibration
diameter_coeff = 0.00782324

motor_slope = 0
motor_intercept = 0
Pd = 11
d_preform = 7
d_spool = 15.2
# Thermistor Constants
RT0 = 100000     # Î©
T0 = 298.15      # K
B = 3977         # K
VCC = 3.3        # Supply voltage
R = 10000        # R=10KÎ©

#Stepper motor initialisation
SPR = 200                                     # Steps per Revolution, from Stepper Data Sheet
RESOLUTION = {'1': (0, 0, 0),                 # (M0, M1, M2)
              '1/2': (1, 0, 0),
              '1/4': (0, 1, 0),
              '1/8': (1, 1, 0),
              '1/16': (0, 0, 1),
              '1/32': (1, 0, 1)}              #microstepping settings
FACTOR = {'1': 1,
          '1/2': 2,
          '1/4': 4,
          '1/8': 8,
          '1/16': 6,
          '1/32': 32}                         #microstepping settings

# delay = 60/rpm/SPR/FACTOR[microstepping]

# # # # # # # DC Motor Initialisation # # # # # # # # #
# DC Motor initialisation
ppr = 1176       # Pulses Per Revolution of the encoder
dcFreq = 2000     #DC motor PWM frequency
fanFreq = 1000    #Fan PWM frequency
tsample = 0.1  # Sampling period for code execution (s)

# Initializing previous values and starting main clock
tstart = time.perf_counter()   #start internal clock
oldtime = 0                    #old DC time
oldpos = 0                     #old DC position
lasttime = 0                   #old stepper time

# Initialize motor PID terms
# PID Parameters (these should be adjusted based on Ku and Tu)
Ku = 0.4  # ultimate gain (example value, adjust accordingly)
Tu = 0.9  # oscillation period (example value, adjust accordingly)
Kp = 0.6 * Ku
Ti = Tu / 2
Td = Tu / 8
Ki = Kp / Ti
Kd = Kp * Td
# setpoint_rpm = 50 # Desired speed
integral = 0
previous_error = 0
# Anti-windup limits
integral_max = 100
integral_min = -100

integral_d = 0
previous_error_d = 0

# Temperature PID constants
Kp_t = 1.4  # Proportional gain
Ki_t = 0.2  # Integral gain
Kd_t = 0.8  # Derivative gain
# Controller output limits
output_t_min = 0
output_t_max = 1
# Number of readings to average
num_readings = 5
temperature_readings = []

# Initialize PID variables
integral_t = 0
previous_error_t = 0
previous_time_t = time.perf_counter()

# Create instance of lists for saving to csv
CSVList = []   # stores data that will be saved as csv file  
time_list = []  # stores time values
diameter_mm_list = []   # stores diameter values
diameter_setpoint_list = []   # stores diameter set point values
temperature_list = []   # stores temperature values
temp_set_point_list = []   # stores temperature set point values
temp_error_list = []   # stores temperature error values
kp_list = []   # stores kp values
ki_list = []   # stores kd values
kd_list = []   # stores kd values
dc_motor_set_speed_list = []   # stores dc motor set speed values
dc_motor_speed_list = []   # stores dc motor speed values
oscillation_ku_list = []   # stores oscilation ku values
period_tu_list = []   # stores oscillation tu values
pid_list = []   # stores PID values
extruder_speed_list = []   # stores extruder speed values
fan_speed_list = []   # stores fan speed values

counter = 0
count = 0
y=0 # counter variable for main loop
list_time2=[]

device_started = False
diameter_started = False
use_binary = False

################################################################
####### Start of Classes #######################################
######### READ AND PLOT DC MOTOR SPEED ################################
class DCMotorPlot(FigureCanvas):
    def __init__(self, parent=None):
        self.figure = Figure()
        self.axes = self.figure.add_subplot(111)
        super(DCMotorPlot, self).__init__(self.figure)
        self.setParent(parent)

        self.axes.set_title('DC Motor Plot')
        self.axes.set_xlabel('Time')
        self.axes.set_ylabel('Speed (RPM)')

        self.line, = self.axes.plot([], [], lw=2, label='Speed')
        self.line_set_point, = self.axes.plot([], [], lw=2, label='Target Speed')
        self.axes.legend()

        self.x_data = []
        self.y_data = []
        self.y_set_point_data = []

    def update_plot(self, x, y, y_set_point):
        self.x_data.append(x)
        self.y_data.append(y)
        self.y_set_point_data.append(y_set_point)

        self.line.set_data(self.x_data, self.y_data)
        self.line_set_point.set_data(self.x_data, self.y_set_point_data)

        self.axes.relim()
        self.axes.autoscale_view(True, True, True)
        self.draw()

######### READ AND PLOT TEMPERATURE ################################
class TemperaturePlot(FigureCanvas):
    def __init__(self, parent=None):
        self.figure = Figure()
        self.axes = self.figure.add_subplot(111)
        super(TemperaturePlot, self).__init__(self.figure)
        self.setParent(parent)

        self.axes.set_title('Temperature Plot')
        self.axes.set_xlabel('Time')
        self.axes.set_ylabel('Temperature (°C)')

        self.line, = self.axes.plot([], [], lw=2, label='Temperature')
        self.line_set_point, = self.axes.plot([], [], lw=2, label='Target Temperature')
        self.axes.legend()

        self.x_data = []
        self.y_data = []
        self.y_set_point_data = []

    def update_plot(self, x, y, y_set_point):
        self.x_data.append(x)
        self.y_data.append(y)
        self.y_set_point_data.append(y_set_point)

        self.line.set_data(self.x_data, self.y_data)
        self.line_set_point.set_data(self.x_data, self.y_set_point_data)

        self.axes.relim()
        self.axes.autoscale_view(True, True, True)
        self.draw()

############ READ AND PLOT FIBER DIAMETER ########################
class DiameterPlot(FigureCanvas):
    def __init__(self, parent=None):
        self.figure = Figure()
        self.axes = self.figure.add_subplot(111)
        super(DiameterPlot, self).__init__(self.figure)
        self.setParent(parent)

        self.axes.set_title('Diameter Plot')
        self.axes.set_xlabel('Time')
        self.axes.set_ylabel('Diameter (mm)')

        self.line, = self.axes.plot([], [], lw=2, label='Diameter - Gray Filter')
        self.line_set_point, = self.axes.plot([], [], lw=2, label='Target Diameter')
#         self.line_binaryimg, = self.axes.plot([], [], lw=2, label='Diameter - Binary Filter')
        self.axes.legend()
        self.x_data = []
        self.y_data = []
        self.y_set_point_data = []
#         self.y_binary_data = []

    def update_plot(self, x, y, y_set_point): #, y_binary):
        self.x_data.append(x)
        self.y_data.append(y)
        self.y_set_point_data.append(y_set_point)
#         self.y_binary_data.append(y_binary)

        self.line.set_data(self.x_data, self.y_data)
        self.line_set_point.set_data(self.x_data, self.y_set_point_data)
#         self.line_binaryimg.set_data(self.x_data, self.y_binary_data)

        self.axes.relim()
        self.axes.autoscale_view(True, True, True)
        self.draw()

########### GPIO SETUP FOR HARDWARE CALLS ######################
class GPIOController:
    def __init__(self):
        
        global heaterPin
        # GPIO Pin Definitions
        self.extruderDirection = 16   # Extruder Direction Pin
        self.extruderStep = 12        # Extruder Step Pin
        self.encoderPin1 = 24         # DC Motor Encoder Pin 1
        self.encoderPin2 = 23         # DC Motor Encoder Pin 2
        self.dcmotorPin = 5           # DC Motor PWM Pin
        heaterPin = 6            # Heater Pin
        self.fanPin = 13              # Fan Pin
        self.M0 = 17                  # Microstepping Pin 1
        self.M1 = 27                  # Microstepping Pin 2
        self.M2 = 22                  # Microstepping Pin 3

        # Initialize GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering

        # Set up GPIO pins
        GPIO.setup(heaterPin, GPIO.OUT)
        GPIO.setup(self.extruderDirection, GPIO.OUT)
        GPIO.setup(self.extruderStep, GPIO.OUT)
        GPIO.setup(self.dcmotorPin, GPIO.OUT)
        GPIO.setup(self.fanPin, GPIO.OUT)
        GPIO.setup((self.M0, self.M1, self.M2), GPIO.OUT)

        # Set initial state (if needed)
        #GPIO.output(self.extruderDirection, GPIO.LOW)  # Replace with your desired initial state
        
        GPIO.output(self.extruderDirection, direction)
       
        # Initialize DC motor encoder and SPI bus
        self.initialize_encoder_and_spi()

        # Stepper motor initialisation
        self.SPR = 200                                     # Steps per Revolution, from Stepper Data Sheet
        self.RESOLUTION = {'1': (0, 0, 0),                 # (M0, M1, M2)
                           '1/2': (1, 0, 0),
                           '1/4': (0, 1, 0),
                           '1/8': (1, 1, 0),
                           '1/16': (0, 0, 1),
                           '1/32': (1, 0, 1)}              # microstepping settings
        self.FACTOR = {'1': 1,
                       '1/2': 2,
                       '1/4': 4,
                       '1/8': 8,
                       '1/16': 16,
                       '1/32': 32}                         # microstepping settings

        self.microstepping = '1/4'  # Default microstepping mode
        self.rpm = 0.6  # Default RPM
        self.set_microstepping(self.microstepping)
    def set_microstepping(self, mode):
        mode_pins = (self.M0, self.M1, self.M2)
        mode_values = self.RESOLUTION[mode]

        for pin, value in zip(mode_pins, mode_values):
            GPIO.output(pin, value)

        self.microstepping = mode
        self.update_delay()
    def set_rpm(self, rpm):
        self.rpm = rpm
        self.update_delay()

    def update_delay(self):
        self.delay = 60 / self.rpm / self.SPR / self.FACTOR[self.microstepping]

    def step(self, direction):
         # Set direction
        GPIO.output(self.extruderDirection, direction)
# 
# #         while True:
#         GPIO.output(self.extruderStep, GPIO.HIGH)
#         time.sleep(self.delay)
#         GPIO.output(self.extruderStep, GPIO.LOW)
#         time.sleep(self.delay)
#         print(self.delay)

    def initialize_encoder_and_spi(self):
        global channel_0
        global encoder
        # Initialise DC motor encoder
        encoder = RotaryEncoder(self.encoderPin1, self.encoderPin2, max_steps=0)

        # Create the SPI bus
        self.spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

        # Create the cs (chip select)
        self.cs = digitalio.DigitalInOut(board.D8)

        # Create the mcp object
        self.mcp = MCP.MCP3008(self.spi, self.cs)

        # Create analog inputs connected to the input pins on the MCP3008
        channel_0 = AnalogIn(self.mcp, MCP.P0)

    def start_fan(self, fan_freq, fan_duty):
        self.fan_pwm = GPIO.PWM(self.fanPin, fan_freq)
        self.fan_pwm.start(fan_duty)

    def update_fan_duty(self, new_duty):
        self.fan_pwm.ChangeDutyCycle(new_duty)

    def stop_fan(self):
        self.fan_pwm.stop()

    def start_dc_motor(self, dc_freq, dc_duty):
        global dc_motor_pwm
        dc_motor_pwm = GPIO.PWM(self.dcmotorPin, dc_freq)
        dc_motor_pwm.start(dc_duty)

    def update_dc_duty(self, new_duty):
        dc_motor_pwm.ChangeDutyCycle(new_duty)

    def stop_dc_motor(self):
        dc_motor_pwm.stop()
    def start_stepper_motor(self, microstepping):
        GPIO.output((self.M0, self.M1, self.M2), self.RESOLUTION[microstepping])
    def start_devices(self, fan_freq, fan_duty, dc_freq, dc_duty, stepper_rpm):
        self.start_fan(fan_freq, fan_duty)
        self.start_dc_motor(dc_freq, dc_duty)
#         self.set_rpm(stepper_rpm)
#         self.step(1)  # Start stepping in the clockwise direction


        
    def cleanup(self):
        GPIO.cleanup()
class PID_Controls():
    
    global fiber_diameter
    def __init__(self):
        super(PID_Controls,self).__init__()
#         self.connection = video_widget.line_value_updated.connect(self.handle_diam)
        #~~~~~~ Get diameter and plot on graph ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Define a global variable to store the diameter value
        self.diameter_value = 0.0
        # Define a global variable to store the diameter set point
        self.diameter_set_point = 0.0  # Set an initial value

        # Connect the line_value_updated signal to the update_ slot
        video_widget.line_value_updated.connect(self.update_line_value)
        
        
    def update_line_value(self, line_value):
        # Update the diameter_value with the new line_value
        self.diameter_value = line_value

        # Call the update_plot function with the current time and diameter value
        current_time = time.time()  # Get the current time
#         diameter_plot.update_plot(current_time, self.diameter_value) #, self.diameter_set_point)
    
    def temperature(self):
        try:
            #~~~~~~~~~~~ CONTROL TEMPERATURE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            global previous_time_t
            global temperature_readings
            global integral_t
            global previous_error_t
            global Kp_t
            global Ki_t
            global Kd_t
            global error_t
            
            slider_value = tempslider.value()
            Kp_t = kpslider.value()
            Ki_t = kislider.value()
            Kd_t = kdslider.value()
            # Simple heater controls
            targetTemp = slider_value
            VR = (channel_0.voltage)
            # with contextlib.suppress(ZeroDivisionError):
            RT = (VCC - VR) * R / VR
            ln = math.log(RT / RT0)
            TX = (1 / ((ln / B) + (1 / T0)))
            TX = TX - 273.15
            current_time_t = time.perf_counter()
            elapsed_time_t = current_time_t - previous_time_t
            previous_time_t = current_time_t
            temperature_readings.append(TX)
            # Turn the heater off if there are not enough readings to prevent building up too much heat
            GPIO.output(heaterPin, GPIO.LOW)
            # Ensure we have the required number of readings
            if len(temperature_readings) >= num_readings:
                # Calculate the average temperature
                average_temp = sum(temperature_readings) / num_readings
                temperature_readings = []  # Reset the readings list
                # Calculate error
                error_t = targetTemp - 1 - average_temp
                # Calculate integral
                integral_t += error_t * elapsed_time_t
                # Calculate derivative
                derivative_t = (error_t - previous_error_t) / elapsed_time_t
                previous_error_t = error_t
                # Compute the PID output
                PID_output_t = Kp_t * error_t + Ki_t * integral_t + Kd_t * derivative_t
                # Apply anti-windup: Clamp the integral term if the PID output is saturated
                if PID_output_t > output_t_max:
                    PID_output_t = output_t_max
                    integral_t -= error_t * elapsed_time_t  # Remove the effect of this error integration
                elif PID_output_t < output_t_min:
                    PID_output_t = output_t_min
                    integral_t -= error_t * elapsed_time_t  # Remove the effect of this error integration
                # Apply the PID output
                if PID_output_t > 0:
                    GPIO.output(heaterPin, GPIO.HIGH)
                else:
                    GPIO.output(heaterPin, GPIO.LOW)
            else:
                error_t =0
                PID_output_t=0
            

            current_time = time.time()  # Get the current time
            temperature_value = TX  # Replace with your actual temperature value
            temperature_set_point = slider_value  # Replace with your desired temperature set point
            temperature_plot.update_plot(current_time, temperature_value, temperature_set_point)
            temp_error_list.append(error_t)   # stores temperature error values
            temperature_list.append(TX)   # stores temperature values
            temp_set_point_list.append(temperature_set_point)   # stores temperature set point values
            kp_list.append(Kp_t)   # stores kp values
            ki_list.append(Ki_t)   # stores kd values
            kd_list.append(Kd_t)   # stores kd values   
            pid_list.append(PID_output_t)   # stores PID values
        except Exception as e:
            print (f"\n An error occurred while updating temperature controls: {e}\n\n")
            QMessageBox.information(app.activeWindow(),"Temperature control didn't update", "The temperature controls didn't update, please try restarting program.")
            pass
        finally:
            pass
    def dc_motor(self):
        try:
            global oldtime
            global oldpos
            global integral
            global previous_error
            global integral_d
            global previous_error_d
            global current_time
            
            diameter_set_point = diameterslider.value()
            stepper_rpm = extrslider.value()
            rpm = extrslider.value()
            delay = 60 / rpm / SPR / FACTOR[microstepping]
            #Stepper motor control
            GPIO.output(16, 1)#direction
            GPIO.output(12, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(12, GPIO.LOW)
            time.sleep(delay)

            # PID Parameters for DC motor control
            Ku = Gainslider.value() 
            Tu = Oscslider.value() 
            Kp = 0.6 * Ku
            Ti = Tu / 2
            Td = Tu / 8
            Ki = Kp / Ti
            Kd = Kp * Td
                                                                                                                                                                                                                                                                                                                                       
            # PID Parameters for diameter control
            Ku_d = gain_dslider.value()  
            Tu_d = osc_dslider.value()  
            Kp_d = 0.6 * Ku_d
            Ti_d = Tu_d / 2
            Td_d = Tu_d / 8
            Ki_d = Kp_d / Ti_d
            Kd_d = Kp_d * Td_d

            current_time = time.perf_counter() - tstart
            
            if len(diameter_mm_list)>=1:
                diameter_value = diameter_mm_list[-1]
            else:
                diameter_value = 0
            if diameter_started == True:
                diameter_plot.update_plot(current_time, diameter_value, diameter_set_point)  
                if len(diameter_mm_list) >= 20:
                    # Get the last 10 items
                    last_measurements = diameter_mm_list[-20:]
                    ma_diameter = sum(last_measurements) / len(last_measurements)
                    error_d = diameter_set_point - ma_diameter
                    integral_d += error_d * 0.35
                    integral_d = max(min(integral_d, 0.5), -0.5)
                    derivative_d = (error_d - previous_error_d) / 0.35
                    output_d2s = Kp_d * error_d + Ki_d * integral_d + Kd_d * derivative_d
                    rpm_diff =diameter_to_spool_rpm(output_d2s, stepper_rpm)        
                    #print(rpm_diff)
                    setpoint_rpm = min(max(rpm_diff, 0), 60)
                    previous_error_d = error_d
                    #print(setpoint_rpm)
                else:
                    print("not enough:")
                    setpoint_rpm = diameter_to_spool_rpm(diameter_set_point, stepper_rpm)   
            else:
                setpoint_rpm = 50 # Desired speed
            # Start of DC motor PID control
            
            # Calculation of DC motor speed-mitcop
            
            
            dt = time.perf_counter() - oldtime           
            ds = encoder.steps - oldpos
            rpm_DC = ds / ppr / dt * 60
            # PID Control
            error = setpoint_rpm - rpm_DC
            integral += error * dt
            integral = max(min(integral, integral_max), integral_min)  # Constrain integral to prevent windup
            derivative = (error - previous_error) / dt
            # Compute PID output in RPM
            output_rpm = Kp * error + Ki * integral + Kd * derivative
            # Convert PID output from RPM to duty cycle
            output_duty = rpm_to_duty_cycle(output_rpm)
            output_duty = min(max(output_duty, 0), 100)  # Constrain duty cycle between 0 and 100
            # Update duty cycle
            gpio_controller.update_dc_duty(output_duty)
            # p.ChangeDutyCycle(output_duty)
            previous_error = error
            
            oldtime = time.perf_counter()
            oldpos = encoder.steps
            # Update fan duty cycle
            gpio_controller.update_fan_duty(fanslider.value())
            # End of DC motor PID control
            
            dcslider_value = slider.value()
                       
            dcspeed_value = rpm_DC  
            dcspeed_set_point = setpoint_rpm
            dcmotor_plot.update_plot(current_time, dcspeed_value, dcspeed_set_point)
            
            time_list.append(time.time())# stores time values
            diameter_setpoint_list.append(self.diameter_set_point)  # stores diameter set point values
            dc_motor_set_speed_list.append(dcspeed_set_point)  # stores dc motor set speed values
            dc_motor_speed_list.append(dcspeed_value)  # stores dc motor speed values
            oscillation_ku_list.append(Ku)   # stores oscilation ku values
            period_tu_list.append(Tu)   # stores oscillation tu values
            extruder_speed_list.append(extrslider.value())  # stores extruder speed values
            fan_speed_list.append(fanDuty)   # stores fan speed values
            
        except Exception as e:
            print (f"\n An error occurred while updating dc motor controls: {e}\n\n")
            QMessageBox.information(app.activeWindow(),"DC motor control didn't update", "The spooling motor controls didn't update, please try restarting program.")
            pass
        finally:
            pass

############ DISPLAY LIVE AND BINARY VIDEO #######################
class VideoWidget(QWidget):
    line_value_updated = pyqtSignal(float)  # Create a new signal

    def __init__(self, parent=None):
        super(VideoWidget, self).__init__(parent)
        self.video_size = None
        self.setup_ui()
        self.setup_camera()

    def setup_ui(self):
        layout = QGridLayout()
        self.video_label = QLabel()
        layout.addWidget(self.video_label, 2, 5, 10, 5)  # Add the VideoWidget to the layout
        self.bvideo_label = QLabel()
        layout.addWidget(self.bvideo_label, 11, 5, 5, 5)  # Add the VideoWidget to the layout
        self.binaryvideo_label = QLabel()
        layout.addWidget(self.binaryvideo_label, 11, 8, 5, 5)  # Add the VideoWidget to the layout
        self.setLayout(layout)

    def setup_camera(self):
        self.cap = cv2.VideoCapture(0)  # Replace 0 with the appropriate camera index

    def show_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Live video
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Display the frame with lines
            img = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
            pix = QPixmap.fromImage(img)
            self.video_label.setPixmap(pix)
            
            # Process gray image
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            
            
            # Process binary image
            gray2 = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            _, binary = cv2.threshold(gray2, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            
            if use_binary == False:
                edges = cv2.Canny(gray, 100, 255, apertureSize=3)
            else:
                edges = cv2.Canny(binary, 100, 255, apertureSize=3)

            # Get diameter from the binary image
            line_value = self.read_line_value(edges)
            # Plot lines on the frame
            frame = self.plot_lines(frame, edges)
            # Emit the line_value_updated signal with the new line_value
            self.line_value_updated.emit(line_value)
            # Update diameter plot
            diameter_mm_list.append(round(float(line_value), 2))  # Stores diameter values
#             if line_value != 0:
#                 diameter_plot.update_plot(current_time, line_value)

            # Display the frame with lines
            img = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
            pix = QPixmap.fromImage(img)
            self.video_label.setPixmap(pix)
            
            # Display the gray image
            img2 = QImage(gray, gray.shape[1], gray.shape[0], QImage.Format_Grayscale8)
            pix2 = QPixmap.fromImage(img2)
            self.bvideo_label.setPixmap(pix2)
            # Binary Image
            img3 = QImage(binary, binary.shape[1], binary.shape[0], QImage.Format_Grayscale8)
            pix3 = QPixmap.fromImage(img3)
            self.binaryvideo_label.setPixmap(pix3)
 
    def calibrate_line_value(self):
        ret, frame = self.cap.read()
        if ret:
            # Live video
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Display the frame with lines
            img = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
            pix = QPixmap.fromImage(img)
            self.video_label.setPixmap(pix)
            
            # Process gray image
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            
            edges = cv2.Canny(gray, 100, 255, apertureSize=3)
            
            lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)
            if lines is not None and len(lines) > 1:
                line_distances = []
                for line in lines:
                    rho, theta = line[0]
                    line_distances.append((rho, theta))
                if len(line_distances) == 0:
                    print("Empty")
                    width_of_wire_mm = 0
                    
                else:
                    max_distance = 0
                    extreme_line1 = None
                    extreme_line2 = None
                    # Calculate the distance between every pair of lines
                    for i in range(len(line_distances)):
                        for j in range(i + 1, len(line_distances)):
                            distance = abs(line_distances[i][0] - line_distances[j][0])
                            if distance > max_distance:
                                max_distance = distance
                                extreme_line1 = line_distances[i]
                                extreme_line2 = line_distances[j]
                    # Calculate the width of the wire in mm (example conversion factor, needs tuning)
                    width_of_wire_mm = max_distance
                    return width_of_wire_mm
            else:
                return None
        return None

 
 
    def read_line_value(self, edges):
        global diameter_coeff
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)
        if lines is not None and len(lines) > 1:
            line_distances = []
            for line in lines:
                rho, theta = line[0]
                line_distances.append((rho, theta))
            if len(line_distances) == 0:
                print("Empty")
                width_of_wire_mm = 0
                
            else:
                max_distance = 0
                extreme_line1 = None
                extreme_line2 = None
                # Calculate the distance between every pair of lines
                for i in range(len(line_distances)):
                    for j in range(i + 1, len(line_distances)):
                        distance = abs(line_distances[i][0] - line_distances[j][0])
                        if distance > max_distance:
                            max_distance = distance
                            extreme_line1 = line_distances[i]
                            extreme_line2 = line_distances[j]

                # Calculate the width of the wire in mm (example conversion factor, needs tuning)
                width_of_wire_mm = max_distance * diameter_coeff
        else:
            width_of_wire_mm = 0
        return width_of_wire_mm

    def plot_lines(self, frame, edges):
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 110)
        
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        return frame

    def closeEvent(self, event):
        self.cap.release()
        event.accept()
################################################################
####### End of Classes. Start Threading and GUI ################
        
########### HARDWARE CONTROL ###########################
def motor_control_thread():
    
    # ~~~~~~~ Initialize variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    global iteration
    iteration = 0
    # ~~~~~~~ Start motors ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    try:
        global gpio_controller
        global control
        
        # Talks to hardware
        gpio_controller = GPIOController() # Initialize controller class
        
        gpio_controller.start_devices(1000, 45, 1000, 45, 0.6)
        # Makes decisions for the hardware, also updates graphs
        control = PID_Controls()
        control.temperature()
        control.dc_motor()
        
    except Exception as e:
        print (f"\n An error occurred while starting motors: {e}\n\n")
        QMessageBox.information(app.activeWindow(),"Motors didn't start", "The motors didn't start, please make sure everything is turned on and in working condition and then try restarting program.")    
        pass
    finally:
        pass
    # ~~~~~~~ Continually update the motors ~~~~~~~~~~~~~~~~~~~    
    while True:
        # This is where the controls and hardware are continuously run
        try:
#             print('boop')
            iteration += 1
            #Make controls decisions 
            MakeControllerDecisions()
            # Control motor
            control_motor(50)
        except Exception as e:
            print (f"\n An error occurred while updating motors: {e}\n\n")
            QMessageBox.information(app.activeWindow(),"Motors didn't update", "The motors didn't update, please make sure everything is turned on and in working condition and then try restarting program.")      
        finally:
            pass
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
def control_motor(duty):
    # Stepper extruder motor
    gpio_controller.set_rpm(extrslider.value())
    gpio_controller.step(1)  # Start stepping in the clockwise direction
def video_update():
    video_widget.show_frame()
def MakeControllerDecisions():
    if device_started == True:
        control.temperature()
        control.dc_motor()
    else:
        pass

    #control.diameter_control()
#     control.diameter()
def update_gui():
#     print('gui')
#     dc_label.setText(f"iter {iteration}")
    video_update()
def gui_thread():
    global app
    app = QApplication([])#sys.argv)
    global window
    window = QWidget()
    global dc_label
#     central_widget = QWidget()
    # ~~~~~~~~~~~ Initialize the Interface ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # # # # # # # First Column # # # # # # # 
    # Add title
    title = QLabel("Diameter Close Loop Controls:")
    title.setStyleSheet("font-size: 18px; font-weight: bold;")

    # Add the "DC Spooling Motor" label
    dc_label = QLabel("DC Spooling Motor")
    dc_label.setStyleSheet("font-size: 16px; font-weight: bold;")
    
    global dcmotor_plot
    dcmotor_plot = DCMotorPlot()

    # Add the "Temperature" label
    temp_label = QLabel("Temperature")
    temp_label.setStyleSheet("font-size: 16px; font-weight: bold;")
    
    global temperature_plot
    temperature_plot = TemperaturePlot()

        
    # Add the "Diameter (mm)" label
    dia_label = QLabel("Diameter")
    dia_label.setStyleSheet("font-size: 16px; font-weight: bold;")
    
    global diameter_plot
    diameter_plot = DiameterPlot()
    
    global binarycheck
    binarycheck = QCheckBox("Binary")
    binarycheck.setStyleSheet("font-size: 14px;")
    binarycheck.stateChanged.connect(checkbox_state_changed)


    # # # # # # # Third Column # # # # # # 
    # Add the "Spooling Motor Set Speed (RPM)" label
    dc_set_label = QLabel("Spooling Motor Set Speed (RPM)")
    dc_set_label.setStyleSheet("font-size: 16px; font-weight: bold;")
    # Add DC set speed slider
    global slider
    slider = QSlider(Qt.Horizontal)
    slider.setMinimum(20)
    slider.setMaximum(60)
    slider.setValue(50)
#     slider.valueChanged.connect(slider_value_changed)
    # Add a label to the slider
    slider_value_label = QLabel(str(slider.value()))

    # Add the "Controller Gain" label
    gain_label = QLabel("DC Motor Gain Ku")
    gain_label.setStyleSheet("font-size: 14px; font-weight: bold;")
    # Add Gain slider
    global Gainslider
    Gainslider = QDoubleSpinBox()
    Gainslider.setMinimum(0.0)
    Gainslider.setMaximum(2.0)
    Gainslider.setValue(0.4)
    Gainslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
    Gainslider.setDecimals(1)  # Set the number of decimal places to show
    # Add a label to the slider
    Gainslider_value_label = QLabel(str(Gainslider.value()))

    # Add the "Oscillation Period" label
    Osc_label = QLabel("DC Motor Oscillation Period Tu")
    Osc_label.setStyleSheet("font-size: 14px; font-weight: bold;")
    # Add Oscillation slider
    global Oscslider
    Oscslider = QDoubleSpinBox()
    Oscslider.setMinimum(0.0)
    Oscslider.setMaximum(2.0)
    Oscslider.setValue(0.9)
    Oscslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
    Oscslider.setDecimals(1)  # Set the number of decimal places to show
    # Add a label to the slider
    Oscslider_value_label = QLabel(format(Oscslider.value(), '.1f'))

    # Add the "Extrusion Motor Speed" label
    extr_label = QLabel("Extrusion Motor Speed (RPM)")
    extr_label.setStyleSheet("font-size: 16px; font-weight: bold;")
    # Add Extrusion speed slider
    global extrslider
    
    extrslider = QDoubleSpinBox()
    extrslider.setMinimum(0.0)
    extrslider.setMaximum(2.0)
    extrslider.setValue(1.2)
    extrslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
    extrslider.setDecimals(1)  # Set the number of decimal places to show
    # Add a label to the slider
    extrslider_value_label = QLabel(format(extrslider.value(), '.1f'))

    # Add the "Temperature (C)" label
    temp_set_label = QLabel("Temperature (C)")
    temp_set_label.setStyleSheet("font-size: 16px; font-weight: bold;")
    # Add Temperature slider
    global tempslider
    tempslider = QSlider(Qt.Horizontal)
    tempslider.setMinimum(65)
    tempslider.setMaximum(105)
    tempslider.setValue(95)
    # Add a label to the slider
    tempslider_value_label = QLabel(str(tempslider.value()))

    # Add the "Kp" label
    kp_label = QLabel("Temperature Kp")
    kp_label.setStyleSheet("font-size: 14px; font-weight: bold;")
    # Add kp slider
    global kpslider
    kpslider = QDoubleSpinBox()
    kpslider.setMinimum(0.0)
    kpslider.setMaximum(2.0)
    kpslider.setValue(1.4)
    kpslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
    kpslider.setDecimals(1)  # Set the number of decimal places to show
#         self.kpslider.valueChanged.connect(self.slider_value_changed)
    # Add a label to the slider
    kpslider_value_label = QLabel(format(kpslider.value(), '.1f'))

    # Add the "Ki" label
    ki_label = QLabel("Temperature Ki")
    ki_label.setStyleSheet("font-size: 14px; font-weight: bold;")
    # Add ki slider
    global kislider
    kislider = QDoubleSpinBox()
    kislider.setMinimum(0.0)
    kislider.setMaximum(2.0)
    kislider.setValue(0.2)
    kislider.setSingleStep(0.1)  # Set the step size for decimal adjustment
    kislider.setDecimals(1)  # Set the number of decimal places to show
    kislider_value_label = QLabel(format(kislider.value(), '.1f'))

    # Add the "Kd" label
    kd_label = QLabel("Temperature Kd")
    kd_label.setStyleSheet("font-size: 14px; font-weight: bold;")
    # Add kd slider
    global kdslider
    kdslider = QDoubleSpinBox()
    kdslider.setMinimum(0.0)
    kdslider.setMaximum(2.0)
    kdslider.setValue(0.8)
    kdslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
    kdslider.setDecimals(1)  # Set the number of decimal places to show
    kdslider_value_label = QLabel(format(kdslider.value(), '.1f'))
    
    # Add the target diameter label
    diameter_label = QLabel("Target Diameter (mm)")
    diameter_label.setStyleSheet("font-size: 16px; font-weight: bold;")    
    # Add diameter slider
    global diameterslider
    diameterslider = QDoubleSpinBox()
    diameterslider.setMinimum(0.3)
    diameterslider.setMaximum(0.6)
    diameterslider.setValue(0.35)
    diameterslider.setSingleStep(0.01)  # Set the step size for decimal adjustment
    diameterslider.setDecimals(2)  # Set the number of decimal places to show
    # Add a label to the slider
    diameterslider_value_label = QLabel(format(diameterslider.value(), '.2f'))
    
    # Add the diameter ku label
    gain_d_label = QLabel("Diameter Gain ku")
    gain_d_label.setStyleSheet("font-size: 14px; font-weight: bold;")    
    # Add diameter slider
    global gain_dslider
    gain_dslider = QDoubleSpinBox()
    gain_dslider.setMinimum(0.1)
    gain_dslider.setMaximum(2)
    gain_dslider.setValue(1.2)
    gain_dslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
    gain_dslider.setDecimals(1)  # Set the number of decimal places to show
    # Add a label to the slider
    gain_dslider_value_label = QLabel(format(gain_dslider.value(), '.1f'))

    # Add the diameter ku label
    osc_d_label = QLabel("Diameter Oscillation Period Tu")
    osc_d_label.setStyleSheet("font-size: 14px; font-weight: bold;")    
    # Add diameter slider
    global osc_dslider
    osc_dslider = QDoubleSpinBox()
    osc_dslider.setMinimum(0.1)
    osc_dslider.setMaximum(2)
    osc_dslider.setValue(0.8)
    osc_dslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
    osc_dslider.setDecimals(1)  # Set the number of decimal places to show
    # Add a label to the slider
    osc_dslider_value_label = QLabel(format(osc_dslider.value(), '.1f'))

    # Add the "Fan Duty Cycle" label
    fan_set_label = QLabel("Fan Duty Cycle (%)")
    fan_set_label.setStyleSheet("font-size: 14px; font-weight: bold;")
    # Add fan slider
    global fanslider
    fanslider = QSlider(Qt.Horizontal)
    fanslider.setMinimum(0)
    fanslider.setMaximum(100)
    fanslider.setValue(30)
#         fanslider.valueChanged.connect(self.slider_value_changed)
    # Add a label to the slider
    fanslider_value_label = QLabel(str(fanslider.value()))

    # Add an editable text box
    global text_box
    text_box = QLineEdit()
    text_box.setText("Enter a file name")

    # # # # # # # Fifth Column # # # # # # 
    global video_widget
    video_widget = VideoWidget() # Add live video
    
    
    
    # Add button
    button = QPushButton("Download CSV File")
    button.clicked.connect(print_button_clicked)
    button.setStyleSheet("background-color: green;font-size: 14px; font-weight: bold;")
    
    # Add button
    global start_diameter_button
    start_diameter_button = QPushButton("Start/stop close loop control")
    start_diameter_button.clicked.connect(start_diameter_button_clicked)
    start_diameter_button.setStyleSheet("background-color: green;font-size: 14px; font-weight: bold;")
    
    # Add button
    global start_device_button
    start_device_button = QPushButton("Start device")
    start_device_button.clicked.connect(start_device_button_clicked)
    start_device_button.setStyleSheet("background-color: green;font-size: 14px; font-weight: bold;")
    
    # Add button
    global calibrate_motor_button
    calibrate_motor_button = QPushButton("Calibrate motor")
    calibrate_motor_button.clicked.connect(calibrate_motor_button_clicked)
    calibrate_motor_button.setStyleSheet("background-color: green;font-size: 14px; font-weight: bold;")
    
    # Add button
    global calibrate_camera_button
    calibrate_camera_button = QPushButton("Calibrate camera")
    calibrate_camera_button.clicked.connect(calibrate_camera_button_clicked)
    calibrate_camera_button.setStyleSheet("background-color: green;font-size: 14px; font-weight: bold;")

    # ~~~~~~~~~~~ Set the Layout ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    layout = QGridLayout()
    # # # # # # # First Column # # # # # # # 
    layout.addWidget(title,0,0,1,3) # Add title
  
    # Add the dc motor plot to the layout
    layout.addWidget(dcmotor_plot, 11,0,8,6)
    
    layout.addWidget(start_diameter_button, 10, 0) # Add button
    
    layout.addWidget(start_device_button, 1, 0) # Add button
    
    layout.addWidget(calibrate_motor_button, 1, 1) # Add button
       
    layout.addWidget(calibrate_camera_button, 1, 2) # Add button
    
    layout.addWidget(binarycheck,10,1)
    
    # Add the temperature plot to the layout
    layout.addWidget(temperature_plot, 19, 0, 8, 6)   
    layout.addWidget(diameter_plot,2,0,8,6)  # Add the plot to the grid layout

    # # # # # # # Third Column # # # # # # # 
    layout.addWidget(diameter_label,2,6) # Add dc set speed label
    layout.addWidget(diameterslider,3,6) # Add dc set speed slider
    layout.addWidget(diameterslider_value_label,3,7) # Add label to dc set speed slider
    diameterslider.valueChanged.connect(lambda value: diameterslider_value_label.setText(str(value))) # update dc set speed slider
    
    layout.addWidget(gain_d_label,4,6) # Add gain label
    layout.addWidget(gain_dslider,5,6) # Add gain slider
    layout.addWidget(gain_dslider_value_label,5,7) # Add label to gain slider
    gain_dslider.valueChanged.connect(lambda value: gain_dslider_value_label.setText(str(format(value,'.1f')))) # update gain slider
       
    layout.addWidget(osc_d_label,6,6) # Add gain label
    layout.addWidget(osc_dslider,7,6) # Add gain slider
    layout.addWidget(osc_dslider_value_label,7,7) # Add label to gain slider
    osc_dslider.valueChanged.connect(lambda value: osc_dslider_value_label.setText(str(format(value,'.1f'))))

    layout.addWidget(gain_label,8,6) # Add gain label
    layout.addWidget(Gainslider,9,6) # Add gain slider
    layout.addWidget(Gainslider_value_label,9,7) # Add label to gain slider
    Gainslider.valueChanged.connect(lambda value: Gainslider_value_label.setText(str(format(value,'.1f'))))
    
    layout.addWidget(Osc_label,10,6) # Add oscillation label
    layout.addWidget(Oscslider,11,6) # Add oscillation slider
    layout.addWidget(Oscslider_value_label,11,7) # Add label to oscillation slider
    Oscslider.valueChanged.connect(lambda value: Oscslider_value_label.setText(str(format(value,'.1f')))) # update oscillation slider
    
    layout.addWidget(extr_label,12,6) # Add extrusino speed label
    layout.addWidget(extrslider,13,6) # Add extrusino speed slider
    layout.addWidget(extrslider_value_label,13,7) # Add label to extrusino slider
    extrslider.valueChanged.connect(lambda value: extrslider_value_label.setText(str(format(value,'.1f')))) # update oscillation slider
    
    layout.addWidget(temp_set_label,14,6) # Add temp label
    layout.addWidget(tempslider,15,6) # Add temp slider
    layout.addWidget(tempslider_value_label,15,7) # Add label to temp slider
    tempslider.valueChanged.connect(lambda value: tempslider_value_label.setText(str(value))) # update dc set speed slider
    
    layout.addWidget(kp_label,16,6) # Add kp label
    layout.addWidget(kpslider,17,6) # Add kp slider
    layout.addWidget(kpslider_value_label,17,7) # Add label to kp slider
    kpslider.valueChanged.connect(lambda value: kpslider_value_label.setText(str(format(value,'.1f')))) # update oscillation slider
    
    layout.addWidget(ki_label,18,6) # Add ki label
    layout.addWidget(kislider,19,6) # Add ki slider
    layout.addWidget(kislider_value_label,19,7) # Add label to ki slider
    kislider.valueChanged.connect(lambda value: kislider_value_label.setText(str(format(value,'.1f')))) # update oscillation slider

    layout.addWidget(kd_label,20,6) # Add kd label
    layout.addWidget(kdslider,21,6) # Add kd slider
    layout.addWidget(kdslider_value_label,21,7) # Add label to kd slider
    kdslider.valueChanged.connect(lambda value: kdslider_value_label.setText(str(format(value,'.1f')))) # update oscillation slider
    
    layout.addWidget(fan_set_label,22,6) # Add temp label
    layout.addWidget(fanslider,23,6) # Add temp slider
    layout.addWidget(fanslider_value_label,23,7) # Add label to temp slider
    fanslider.valueChanged.connect(lambda value: fanslider_value_label.setText(str(value))) # update dc set speed slider

    layout.addWidget(video_widget.video_label, 2, 8, 11, 1)  # Add the video_label to the layout

    layout.addWidget(video_widget.bvideo_label, 13, 8, 5,1)  # Add the video_label to the layout

    layout.addWidget(video_widget.binaryvideo_label, 18, 8, 5,1)  # Add the video_label to the layout

    layout.addWidget(text_box,24,6) # Add editable text box

    layout.addWidget(button, 24, 8) # Add button
        
    # ~~~~~~ Show the Layout ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    window.setLayout(layout)
 

    window.setWindowTitle("FredBerry Pi - 07252024") # Name of Window
#     window.resize(800, 1900)  # Set window size (width, height)
    window_width = 1400
    window_height = 800
    window.setGeometry(100,100,window_width,window_height)
    window.setFixedSize(window_width,window_height)
    window.setAutoFillBackground(True)
    
    # ~~~~~~~~~~~ Start threading the GUI and motor control ~~~~~~~~~~~    
    timer = QTimer()
    timer.timeout.connect(update_gui)
    timer.start(50)  # Update every 30 milliseconds
    
    motor_thread = threading.Thread(target=motor_control_thread)
    motor_thread.start()
    threading.Lock()
    
    # Begin application
    window.show()
    app.exec_()
    
    motor_thread.join()
def print_button_clicked():
    # Add your code here to handle the button click event
    QMessageBox.information(app.activeWindow(), "CSV Printed", "CSV file printed to working folder")
    testname = text_box.text()        
    csv_file_path = f'Fred_Data_{testname}_.csv'
    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([ 'Time:', 'Diameter mm:','Diameter Set Point:','Temp C:','Temp Set Point:','Temp Error:','Kp:','Ki:','Kd:','DC Motor set Speed (RPM):','DC Motor speed:','Oscillation Ku:','Period Tu:','PID_output:','Extruder Motor speed:','Fan duty cycle:'])
        rows=[]
        print(f'Printed data to csv file with total data points = {len(time_list)}')
        for i in range(len(time_list)-1):
            rows.append([time_list[i],diameter_mm_list[i],diameter_setpoint_list[i],temperature_list[i],temp_set_point_list[i],temp_error_list[i],kp_list[i],ki_list[i],kd_list[i],dc_motor_set_speed_list[i],dc_motor_speed_list[i],oscillation_ku_list[i],period_tu_list[i],pid_list[i],extruder_speed_list[i],fan_speed_list[i]])
    
        writer.writerows(rows)
        
def start_diameter_button_clicked():
    global diameter_started
    if diameter_started == True:
        diameter_started = False
        QMessageBox.information(app.activeWindow(), "Calculations Stopped", "Close loop controls deactivated, diameter measurement paused.")
    else:
        diameter_started = True
        QMessageBox.information(app.activeWindow(), "Calculations Started", "Close loop controls activated, diameter measurement started.")

def start_device_button_clicked():
    global motor_slope, motor_intercept
    global device_started

    if device_started == True:
        pass
    else:
        device_started = True
        
    # Show a message box indicating that calibration is in progress
    root = tk.Tk()
    root.withdraw()  # Hide the main window
    progress_message = QMessageBox.information(app.activeWindow(), "Start", "The device is starting.")
    try:
        # Read the calibration parameters from the file
        desktop_path_motor = "/home/pi/Desktop/motor_calibration.txt"
        desktop_path_camera = "/home/pi/Desktop/camera_calibration.txt"
        if not os.path.isfile(desktop_path_motor):
            QMessageBox.information(app.activeWindow(), "Calibration", "Motor calibration hasn't been completed")
            return  
        with open(desktop_path_motor, "r") as file:
            lines = file.readlines()
            if not lines:
                QMessageBox.information(app.activeWindow(), "Calibration", "Motor calibration hasn't been completed")
                return
            
            for line in lines:
                if "slope" in line:
                    motor_slope = float(line.split(":")[1].strip())
                elif "intercept" in line:
                    motor_intercept = float(line.split(":")[1].strip())
        
#         # Close the progress message box
#         root.update_idletasks()  # Ensure any pending operations are performed
#         root.destroy()  # Close the progress message box
        
        if not os.path.isfile(desktop_path_camera):
            QMessageBox.information(app.activeWindow(), "Calibration", "Camera calibration hasn't been completed")
            return  
        with open(desktop_path_camera, "r") as file:
            lines = file.readlines()
            if not lines:
                QMessageBox.information(app.activeWindow(), "Calibration", "Camera calibration hasn't been completed")
                return
            
            for line in lines:
                diameter_coeff = float(line.split(":")[1].strip())
        
        # Close the progress message box
        root.update_idletasks()  # Ensure any pending operations are performed
        root.destroy()  # Close the progress message box
        
    except Exception as e:
        messagebox.showerror("Error", f"An error occurred: {e}")
        root.destroy()
        return
    
    finally:
        root.quit()
        #print(f"Calibration completed: slope = {motor_slope}, intercept = {motor_intercept}, diameter coefficient = {diameter_coeff}")
    
    
        
def calibrate_motor_button_clicked():
    # Lists to hold RPM and corresponding duty cycle values
    rpm_values = []
    duty_cycles = []

    num_samples = 5 # Number of samples to average

    try:
        for dc in range(20, 101, 10):  # Sweep duty cycle from 0% to 100% in increments of 10%
            rpm_samples = []
            for _ in range(num_samples):
                gpio_controller.update_dc_duty(dc)
                time.sleep(2)  # Wait for the motor to stabilize
                # Measure RPM
                oldtime = time.perf_counter()
                oldpos = encoder.steps
                time.sleep(tsample)
                newtime = time.perf_counter()
                newpos = encoder.steps
                dt = newtime - oldtime
                ds = newpos - oldpos
                rpm = ds / ppr / dt * 60
                rpm_samples.append(rpm)
            avg_rpm = sum(rpm_samples) / num_samples
            duty_cycles.append(dc)
            rpm_values.append(avg_rpm)
            print(f"Duty Cycle: {dc}% -> Avg RPM: {avg_rpm:.2f}")
        # Fit a curve to the data (e.g., polynomial)
        coefficients = np.polyfit(rpm_values, duty_cycles, 1)  # Fit a 1st-degree polynomial
        slope = coefficients[0]
        intercept = coefficients[1]

        # Save the coefficients to a text file on the Raspberry Pi desktop
        desktop_path = "/home/pi/Desktop/motor_calibration.txt"
        with open(desktop_path, "w") as file:
            file.write(f"motor_slope: {slope:.4f}\n")
            file.write(f"motor_intercept: {intercept:.4f}\n")

    except KeyboardInterrupt:
        print("\nData collection stopped\n\n")

    finally:
        #gpio_controller.cleanup()
        #gpio_controller.stop_dc_motor()
        pass
    QMessageBox.information(app.activeWindow(), "Calibration", "Motor calibration completed. Please restart the program.")
    #gpio_controller.cleanup()
    gpio_controller.stop_dc_motor()
        
def calibrate_camera_button_clicked():
    global diameter_coeff
    num_samples = 20
    total_width = 0
    valid_samples = 0

    for _ in range(num_samples):
        width = video_widget.calibrate_line_value()
        if width is not None:
            total_width += width
            valid_samples += 1

    if valid_samples > 0:
        average_width = total_width / valid_samples
    else:
        average_width = 0
    
    print(f"Average width of wire: {average_width} mm")
    
    diameter_coeff = 0.45/average_width
    print(f"Diameter_coeff: {diameter_coeff} mm")
    
    desktop_path = "/home/pi/Desktop/camera_calibration.txt"
    with open(desktop_path, "w") as file:
        file.write(f"Diameter coefficient: {diameter_coeff:.8f}\n")
    #return average_width
    QMessageBox.information(app.activeWindow(), "Calibration", "Camera calibration completed. Please restart the program.")    


def checkbox_state_changed(state):
    global use_binaryb
    if state == Qt.Checked:
        print("check")
        use_binary = True
    else:
        print("uncheck")
        use_binary = False
def rpm_to_duty_cycle(rpm_DC):
    global motor_slope, motor_intercept
    return  motor_slope* rpm_DC - motor_intercept 
    #     needs to be changed based on calibration results
    
def diameter_to_spool_rpm(diameter, stepper_rpm):
    return  25/28 * Pd * stepper_rpm * d_preform**2 / (d_spool * diameter**2) #rpm is speed of the extruder




if __name__ == "__main__":  
    # Start GUI thread
    gui_thread()
    
