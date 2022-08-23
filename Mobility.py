# -*- coding: utf-8 -*-
"""
Created on Wed Oct 13 17:46:27 2021

@author: nikis
"""

import RPi.GPIO as GPIO     # Import GPIO module      
import time                 # Import time module

GPIO.setmode(GPIO.BCM)      # Set the GPIO pin naming convention to BCM

AIn1 = 24 # orange
AIn2 = 23 # red
BIn1 = 22 # black
BIn2 = 27 # grey

PWMA = 12 # green
PWMB = 13 # brown

# Set up GPIO pins as outputs
GPIO.setup(AIn1,GPIO.OUT) #AIn1
GPIO.setup(AIn2,GPIO.OUT) #AIn2
GPIO.setup(BIn1,GPIO.OUT) #BIn1
GPIO.setup(BIn2,GPIO.OUT) #BIn2
GPIO.setup(PWMA,GPIO.OUT) #PWMA
GPIO.setup(PWMB,GPIO.OUT) #PWMB
GPIO.output(AIn1, GPIO.LOW)
GPIO.output(AIn2, GPIO.LOW)
GPIO.output(BIn1, GPIO.LOW)
GPIO.output(BIn2, GPIO.LOW)    
motorA=GPIO.PWM(PWMA,1000)
motorB=GPIO.PWM(PWMB,1000)
motorA.start(0)
motorB.start(0)

def speed(leftMotor, rightMotor):
    motorA.ChangeDutyCycle(abs(leftMotor))
    if leftMotor > 0:
        GPIO.output(AIn1,GPIO.LOW)  # Set GPIO pin to digital low (off)
        GPIO.output(AIn2,GPIO.HIGH) # Set GPIO pin to digital high (on)
    elif leftMotor < 0:
        GPIO.output(AIn1,GPIO.HIGH)
        GPIO.output(AIn2,GPIO.LOW)
    else:
        GPIO.output(AIn1,GPIO.LOW)
        GPIO.output(AIn2,GPIO.LOW)
        
    motorB.ChangeDutyCycle(abs(rightMotor))       
    if rightMotor > 0:
        GPIO.output(BIn1,GPIO.LOW)
        GPIO.output(BIn2,GPIO.HIGH)
    elif rightMotor < 0:
        GPIO.output(BIn1,GPIO.HIGH)
        GPIO.output(BIn2,GPIO.LOW)
    else:
        GPIO.output(BIn1,GPIO.LOW)
        GPIO.output(BIn2,GPIO.LOW)
     
        
     # Testing with LED
#while(TRUE)
    #left = int(raw_input("LSpeed: "))
    #right = int(raw_input("RSpeed: "))
    #motor_speed(left, right)
    #x = raw_input("Brake: yes/no?")
    #if x == 'y' :
     #     GPIO.cleanup()
      #    break
      #else:
       #   print("Good")
        
                    
