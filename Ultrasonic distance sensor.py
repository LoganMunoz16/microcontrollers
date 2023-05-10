from machine import Pin
import utime

#Declaration
led = Pin(15, Pin.OUT)
trigger = Pin(16, Pin.OUT)
echo = Pin(17, Pin.IN)

def ultra():
    trigger.low()
    utime.sleep_us(2)
    trigger.high()
    utime.sleep_us(5)
    trigger.low()
    while echo.value() == 0:
        signaloff = utime.ticks_us()
    while echo.value() == 1:
        signalon = utime.ticks_us()
    timepassed = signalon - signaloff #Calculate the on and off signal
    distance = (timepassed * 0.0343) / 2 #Calculate the distance - speed of sound?
    print("Object at", distance, "cm")

    if distance < 15:
        led.high()
    else:
        led.low()

while True:
    ultra()
    utime.sleep(1)