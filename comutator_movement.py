import sys
sys.path.append("Motors")
from Motor_Setup_Rabbit import *
import RPi.GPIO as GPIO			# using Rpi.GPIO module
from time import sleep			# import function sleep for delay

"""
Ziegler-Nichols tuning, using the oscillation method.
"""

kc=10                           #propotional gain
pc=5                            #period of oscillation

kp = 0.6*kc                     #propotional error constant
kd = 0.5*pc                     #differentional error constant
ki = 0.125*pc                   #integral error constant

global priError                 #to get rate of change of error
global toError                  #to get total error
priError = 0
toError = 0

""""
This programe used to demonstare how to use Loch Antiphase with Hat-MDD10
AN pin will act as sterring to control direction
DIG pin will act to ON/OFF motor output.
"""

GPIO.setmode(GPIO.BCM)			# GPIO numbering
GPIO.setwarnings(False)			# enable warning from GPIO
AN1 = 12				        # set pwm1 pin on MD10-hat
DIG1 = 26				        # set dir1 pin on MD10-Hat
GPIO.setup(AN1, GPIO.OUT)		# set pin as output
GPIO.setup(DIG1, GPIO.OUT)		# set pin as output
sleep(1)				        # delay for 1 seconds
p1 = GPIO.PWM(DIG1, 100)		# set pwm for M1

def mapping(value, x, y, a, b):
    res = abs(b-a)/(y-x)
    return res*value


def PID_rabbit(y):
    print("Aligning rabbit")
    if y != None:
        curr_y = y
        set_y = 0
        error = set_y - curr_y
        Pvalue = error * kp
        Ivalue = toError * ki
        Dvalue = (error - priError) * kd
        PIDvalue = Pvalue + Ivalue + Dvalue
        priError = error
        toError = error + toError
        print(PIDvalue)
        res_val = mapping(PIDvalue, 0, 60, 0, y)
        if PIDvalue > 0:
            DOWN(res_val)
        elif PIDvalue < 0:
            UP(res_val)
        else:
            STOP()



def UP(s):
    print("UP")			            # display "Forward" when programe run
    GPIO.output(AN1, GPIO.HIGH)		# set AN1 as HIGH, M1B will turn ON
    p1.start(s)				        # set Direction for M

def DOWN(s):
    print("UP")			            # display "Forward" when programe run
    GPIO.output(AN1, GPIO.HIGH)		# set AN1 as HIGH, M1B will turn ON
    p1.start(s)				        # set Direction for M

def STOP():
    print("STOP")
    GPIO.output(AN1, GPIO.LOW)           # set AN1 as LOW, M1B will STOP
    p1.start(0)                          # Direction can ignore
    sleep(1)                             #delay for 1 second