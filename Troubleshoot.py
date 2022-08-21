import os
import sys, getopt
import signal
import time
import RPi.GPIO as GPIO
import serial

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18, GPIO.IN)
GPIO.setup(17, GPIO.IN)
GPIO.setup(27, GPIO.IN)
GPIO.setup(24, GPIO.IN)
g = GPIO.input(18)
i = GPIO.input(27)
h = GPIO.input(17)
k = GPIO.input(24)
b = 0
# Motor Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.output(22, GPIO.LOW)
GPIO.output(23, GPIO.LOW)
servo1 = GPIO.PWM(26, 50)  # Servo initialize
servo1.start(0)


def forward():
    GPIO.output(22, GPIO.HIGH)
    GPIO.output(23, GPIO.LOW)


def reverse():
    GPIO.output(22, GPIO.LOW)
    GPIO.output(23, GPIO.HIGH)


def motor():
    servo1.start(0)
    servo1.ChangeDutyCycle(7)
    time.sleep(0.5)
    servo1.ChangeDutyCycle(0)
    time.sleep(1.5)

    # turn back to 0 degrees
    servo1.ChangeDutyCycle(2)
    time.sleep(0.5)
    servo1.ChangeDutyCycle(0)
    servo1.stop()

while True:
    a = input("Troubleshoot : ")
    b = 2
    c = 1
    if (c == a):
        print(a)
        g = GPIO.input(18)
        h = GPIO.input(17)

        while (g == True):
            reverse()
            g = GPIO.input(18)

        GPIO.output(22, GPIO.LOW)
        GPIO.output(23, GPIO.LOW)
        time.sleep(1)
        motor()

        GPIO.output(26, GPIO.LOW)

        g = GPIO.input(18)
        h = GPIO.input(17)
        while (h == True):
            forward()
            h = GPIO.input(17)

        GPIO.output(22, GPIO.LOW)
        GPIO.output(23, GPIO.LOW)
    elif (b == a):
        print(a)
        i = GPIO.input(27)
        h = GPIO.input(17)

        print(i)
        while (i == True):
            reverse()
            i = GPIO.input(27)

        GPIO.output(22, GPIO.LOW)
        GPIO.output(23, GPIO.LOW)
        time.sleep(1)
        motor()

        i = GPIO.input(27)
        h = GPIO.input(17)
        while (h == True):
            forward()
            h = GPIO.input(17)

        GPIO.output(22, GPIO.LOW)
        GPIO.output(23, GPIO.LOW)
