from machine import Pin, PWM, ADC
import array
import utime
import rp2

# Light Sensor (LDR) Class
class LightSensor():
    def __init__(self, pin):
        self.adc_pin = ADC(pin)
        self.calibrate_light()
        
    def get_lux(self):
        return self.adc_pin.read_u16()
    
    def calibrate_light(self):
        print("Light calibration in progress...")
        sum = 0
        for i in range(0,10):
            sum = sum + self.adc_pin.read_u16()
            utime.sleep(0.5)
            
        self.LIGHT_READING_UPPER_LIMIT = (sum / 10.0) + 2000
        self.LIGHT_READING_LOWER_LIMIT = (sum / 10.0) - 2000
        print("UPPER:", self.LIGHT_READING_UPPER_LIMIT)
        print("LOWER:", self.LIGHT_READING_LOWER_LIMIT)
        
    
    # We will have to fine tune this function
    def light_is_on(self):
        # get light level reading
        light_level = self.adc_pin.read_u16()
        # compare reading to lower and upper limit
        if (light_level < self.LIGHT_READING_LOWER_LIMIT or light_level > self.LIGHT_READING_UPPER_LIMIT):
            return True
        else:
            return False

# END_CLASS

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()

class RGBLED():
    
    def __init__(self, num_leds=1, pin_num=6, brightness=0.2):
        self.num_leds = num_leds
        self.pin_num = pin_num
        self.brightness = brightness
        self.ar = array.array("I", [0 for _ in range(self.num_leds)])
        self.sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(self.pin_num))
        self.sm.active(1)

    ##########################################################################
    def pixels_show(self):
        dimmer_ar = array.array("I", [0 for _ in range(self.num_leds)])
        for i,c in enumerate(self.ar):
            r = int(((c >> 8) & 0xFF) * self.brightness)
            g = int(((c >> 16) & 0xFF) * self.brightness)
            b = int((c & 0xFF) * self.brightness)
            dimmer_ar[i] = (g<<16) + (r<<8) + b
        self.sm.put(dimmer_ar, 8)
        utime.sleep_ms(10)

    def pixels_set(self, i, color):
        self.ar[i] = (color[1]<<16) + (color[0]<<8) + color[2]

    def pixels_fill(self, color):
        for i in range(len(self.ar)):
            self.pixels_set(i, color)

        

#Declaration
led = RGBLED()
trigger = Pin(16, Pin.OUT)
echo = Pin(17, Pin.IN)
buzzer = PWM(Pin(20))
buzzer.duty_u16(0)
light_sensor = LightSensor(27)

def toggle_alarm():
    buzzer.duty_u16(32767)
    buzzer.freq(1000)
    led.pixels_fill((255,0,0))
    led.pixels_show()
    utime.sleep(0.5)
    buzzer.freq(500)
    led.pixels_fill((0,0,255))
    led.pixels_show()
    utime.sleep(0.5)
    buzzer.duty_u16(0)
    led.pixels_fill((0,0,0))
    led.pixels_show()

def alarm():
    for i in range(0,10):
        toggle_alarm()

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
    print("Light reading:", light_sensor.get_lux())

    if distance < 100 or light_sensor.light_is_on():
        alarm()
    else:
        led.pixels_fill((0,0,0))
        led.pixels_show()

while True:
    buzzer.duty_u16(0)
    utime.sleep(1)
    ultra()
    utime.sleep(0.5)

