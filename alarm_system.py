from machine import Pin, PWM
import utime

red = Pin(0, Pin.OUT)
blue = Pin(1, Pin.OUT)
buzzer = PWM((Pin(28)))

while True:
    buzzer.duty_u16(32767)
    red.value(1)
    blue.value(0)
    buzzer.freq(1000)
    utime.sleep(1)
    blue.value(1)
    red.value(0)
    buzzer.freq(500)
    utime.sleep(1)
