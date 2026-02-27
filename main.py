#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor
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
front_left_motor = Motor(Port.A)
front_right_motor = Motor(Port.D)
back_left_motor = Motor(Port.B)
back_right_motor = Motor(Port.C)

# Initialize sensors
# laser_sensor = LaserSensor(Port.S1)
# color_sensor = ColorSensor(Port.S3)
# gyro_sensor = GyroSensor(Port.S4)

# Declare variables
armDirection = "not set"
targetAngle = 0 # degrees
speed = 200 # mm/s

# The DriveBase is composed of two motors, with a wheel on each motor.
# The wheel_diameter and axle_track values are used to make the motors
# move at the correct speed when you give a motor command.
# The axle track is the distance between the points where the wheels
# touch the ground.

# robot = DriveBase(front_left_motor, front_right_motor, wheel_diameter=55.5, axle_track=104) #104 -> 119?
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

def resetWheelAngles():
    front_left_motor.reset_angle(0)
    front_right_motor.reset_angle(0)
    back_left_motor.reset_angle(0)
    back_right_motor.reset_angle(0)

def brake():
    front_left_motor.brake()
    front_right_motor.brake()
    back_left_motor.brake()
    back_right_motor.brake()

def drive(speed, angle):
    front_left_motor.run(-speed)
    front_right_motor.run(-speed)
    back_left_motor.run(speed)
    back_right_motor.run(speed)

def startTurn(speed):
    front_left_motor.run(1 * -speed)
    front_right_motor.run(-1 * -speed)
    back_left_motor.run(1 * speed)
    back_right_motor.run(-1 * speed)

def start():
    brake()
    # gyro_sensor.reset_angle(0)
    resetWheelAngles()
    ev3.speaker.set_volume(20)
    playNote("A")
    playNote("C#")
    playNote("E")






# CODE BELOW

start()

drive(1000,0)
wait(1000)
drive(-1000,0)
wait(1000)
brake()

# startTurn(500)
# wait(1000)
# startTurn(-500)
# wait(1000)

# front_left_motor.run(-400)
# front_right_motor.run(200)
# back_left_motor.run(400)
# back_right_motor.run(-200)
# wait(1000)