import RPi.GPIO as gpio
import time

gpio.setmode(gpio.BCM)

# motor 
motorA = 16
motorB = 20
motorE = 21

gpio.setup(motorA, gpio.OUT)
gpio.setup(motorB, gpio.OUT)
gpio.setup(motorE, gpio.OUT)
# servo
servoPin = 17
gpio.setup(servoPin, gpio.OUT)

# stepper
stepperPins = [5,6,13,19] # controller inputs: in1, in2, in3, in4
for pin in stepperPins:
  gpio.setup(pin, gpio.OUT, initial=0)

dcMin = 2
dcMax = 15

servopwm = gpio.PWM(servoPin, 50) 
servopwm.start(0)

ccw = [ [1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],
        [0,0,1,0],[0,0,1,1],[0,0,0,1],[1,0,0,1] ]
cw = ccw[:]
cw.reverse()

def loop_motor():
  gpio.output(motorA,gpio.HIGH)
  gpio.output(motorB,gpio.LOW)
  gpio.output(motorE,gpio.HIGH)
  time.sleep(2)
  gpio.output(motorA,gpio.LOW)
  gpio.output(motorB,gpio.HIGH)
  gpio.output(motorE,gpio.HIGH)
  time.sleep(2)

def delay_us(tus): # use microseconds to improve time resolution
  endTime = time.time() + float(tus)/ float(1E6)
  while time.time() < endTime:
    pass

def loop(dir): # dir = rotation direction (cw or ccw)
  for i in range(512): # full revolution (8 cycles/rotation * 64 gear ratio)
    for halfstep in range(8): # 8 half-steps per cycle
      for pin in range(4):    # 4 pins that need to be energized
        gpio.output(stepperPins[pin], dir[halfstep][pin])
      delay_us(1000)


try:
  while True:
    for dc in range(dcMin,dcMax):
      servopwm.ChangeDutyCycle(dc)
      print(dc)
      time.sleep(0.1)
    time.sleep(2)
    for dc in range(dcMin,dcMax):
      servopwm.ChangeDutyCycle(dc)
      print(dc)
      time.sleep(0.1)

    loop(cw)
    loop(ccw)

    loop_motor()
except KeyboardInterrupt:
  print("closing")
gpio.cleanup()