#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor, TouchSensor
from pybricks.parameters import Port,Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.iodevices import I2CDevice
import time


class LaserSensor:
    def __init__(self, port):
        self.i2c = I2CDevice(port, 0x02 >> 1)
        self.last_time = 0
        self.last_dist = 0

    def distance(self):
        now = time.time()
        if now - self.last_time > 0.08:
            self.last_time = now
            results = self.i2c.read(0x42, 2)
            self.last_dist = results[0] + (results[1] << 8)
        return self.last_dist


# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize motors
# Left/Right, Side/Middle
LS_motor = Motor(Port.A)
LM_motor = Motor(Port.B)
RS_motor = Motor(Port.D)
RM_motor = Motor(Port.C)

# Initialize sensors
laser_sensor = LaserSensor(Port.S1)
touch_sensor = TouchSensor(Port.S2)
gyro_sensor = GyroSensor(Port.S3)
color_sensor = ColorSensor(Port.S4)

# Declare variables
# armDirection = "not set"
targetAngle = 0 # degrees
# speed = 200 # mm/s

# The DriveBase is composed of two motors, with a wheel on each motor.
# The wheel_diameter and axle_track values are used to make the motors
# move at the correct speed when you give a motor command.
# The axle track is the distance between the points where the wheels
# touch the ground.

# robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104) #104 -> 119?
#robot.settings()

# At start
ev3.speaker.set_volume(20); #ev3.speaker.beep(660,200)
#print(arm_motor.angle())


# Functions

def playNote(note):
    if (note == "A"):
        ev3.speaker.beep(440)
    if (note == "A#"):
        ev3.speaker.beep(466.1637615)
    if (note == "B"):
        ev3.speaker.beep(493.8833013)
    if (note == "C"):
        ev3.speaker.beep(523.2511306)
    if (note == "C#"):
        ev3.speaker.beep(554.365262)
    if (note == "D"):
        ev3.speaker.beep(587.3295358)
    if (note == "D#"):
        ev3.speaker.beep(622.2539674)
    if (note == "E"):
        ev3.speaker.beep(659.2551138)
    if (note == "F"):
        ev3.speaker.beep(698.4564629)
    if (note == "F#"):
        ev3.speaker.beep(739.9888454)
    if (note == "G"):
        ev3.speaker.beep(783.990872)
    if (note == "G#"):
        ev3.speaker.beep(830.6093952)
    if (note == "A5"):
        ev3.speaker.beep(880)

def printLaserDistance():
    print("Laser distance: " + str(laser_sensor.distance()) + "mm")

def printGyroAngle():
    print("Gyro: " + str(gyro_sensor.angle()) + "°")

def printColorReflection():
    print("Reflection: " + str(color_sensor.reflection()))

def resetWheelAngles():
    LS_motor.reset_angle(0)
    LM_motor.reset_angle(0)
    RS_motor.reset_angle(0)
    RM_motor.reset_angle(0)

def brake():
    LS_motor.brake()
    LM_motor.brake()
    RS_motor.brake()
    RM_motor.brake()

def startTurn(speed):
    LS_motor.run(1 * -speed)
    LM_motor.run(1 * -speed)
    RS_motor.run(-1 * -speed)
    RM_motor.run(-1 * -speed)

def drive(speed):
    # if (speed > 100): speed = 100
    # if (speed < -100): speed = -100
    LS_motor.run(-speed)
    LM_motor.run(-speed)
    RS_motor.run(-speed)
    RM_motor.run(-speed)
    

def start():
    reset()
    playNote("A")
    playNote("C#")
    playNote("E")

def reset():
    brake()
    gyro_sensor.reset_angle(0)
    resetWheelAngles()
    ev3.speaker.set_volume(20)
    playNote("A")
    playNote("A")
    playNote("A")



startDelay = 5000

# CODE BELOW
reset()

while (touch_sensor.pressed() == False):
    continue
wait(startDelay)

start()

drive(1000)
wait(700)
brake()
wait(100)

startTurn(1000)
wait(600)

# brake()
# wait(1000)

startTurn(170) # turn
while (laser_sensor.distance() > 1000):
    printLaserDistance()
    continue
wait(50)
brake()

wait(50)

startTurn(-80) # turn back
while (laser_sensor.distance() > 1000):
    printLaserDistance()
    continue
brake()
wait(100)
drive(1000)
time.sleep(5)



# drive(1000)
# wait(800)
# drive(300)

# while (color_sensor.reflection() < 20):
#     printColorReflection()
#     continue
# brake()

# gyro_sensor.reset_angle(0)
# startTurn(1000)
# while (gyro_sensor.angle() < 70):
#     continue
# brake()

# drive(-150,60)

# driveNow = False
# for i in range(650):
#     printLaserDistance()
#     if (laser_sensor.distance() < 1000):
#         driveNow = True
#         break
#     wait(1)

# if (not driveNow):
#     startTurn(80)
#     while (laser_sensor.distance() > 700):
#         continue

# drive(1000,0)
# wait(1000)
