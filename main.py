from machine import Pin, PWM, ADC
import array
import utime
import rp2
from mfrc522 import MFRC522
from esp8266 import ESP8266

class WiFi():
    def __init__(self):
        self.wifi = ESP8266()
        self.esp8266_at_ver = None
        
    def connect(self):
        self.wifi.connectWiFi("LoganPhone","password")
        
    def disconnect(self):
        self.wifi.disconnectWiFi()
        
    def send_alert(self):
        self.httpCode, self.httpRes = self.wifi.doHttpGet("maker.ifttt.com","/trigger/alarm_trigger/json/with/key/yPlK3vBbmFurEZRSVqjW8S3UmsyDzaGd8wsY6Y5kfW","RaspberryPi-Pico", port=80)
        
class RFID():
    def __init__(self):
        self.reader = MFRC522(spi_id=0, sck=2, miso=4, mosi=3, cs=1, rst=0)
        self.registered_uids = [[0x03, 0x5A, 0x0C, 0x0E]]

    def scan_card(self):
        print("Trying to read a card")
        self.reader.init()
        (stat, tag_type) = self.reader.request(self.reader.REQIDL)
        if stat == self.reader.OK:
            (stat, uid) = self.reader.SelectTagSN()

            if stat == self.reader.OK:
                print("Card detected {}  uid={}".format(hex(int.from_bytes(bytes(uid), "little", False)).upper(), self.reader.tohexstring(uid)))

                if uid in self.registered_uids:
                    return True
                else:
                    return False
        else:
            return False


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
            
        self.LIGHT_READING_UPPER_LIMIT = (sum / 10.0) + 500
        self.LIGHT_READING_LOWER_LIMIT = (sum / 10.0) - 500
        print("UPPER:", self.LIGHT_READING_UPPER_LIMIT)
        print("LOWER:", self.LIGHT_READING_LOWER_LIMIT)
        
    
    # We will have to fine tune this function
    def light_is_on(self):
        # get light level reading
        light_level = self.adc_pin.read_u16()
        print("Light level is ", light_level, "lux")
        # compare reading to lower and upper limit
        if (light_level < self.LIGHT_READING_LOWER_LIMIT or light_level > self.LIGHT_READING_UPPER_LIMIT):
            return True
        else:
            return False

# END_CLASS

class DistanceSensor():
    def __init__(self, trigger_pin, echo_pin):
        self.trigger = Pin(trigger_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.calibrate_distance()
        
    def get_distance(self):
        self.trigger.low()
        utime.sleep_us(2)
        self.trigger.high()
        utime.sleep_us(5)
        self.trigger.low()
        while self.echo.value() == 0:
            signaloff = utime.ticks_us()
        while self.echo.value() == 1:
            signalon = utime.ticks_us()
        timepassed = signalon - signaloff #Calculate the on and off signal
        self.distance = (timepassed * 0.0343) / 2 #Calculate the distance - speed of sound?
        return self.distance
    def calibrate_distance(self):
        print("Distance calibrating in progress...")
        sum = 0
        for i in range(0,10):
            sum += self.get_distance()
            utime.sleep(0.5)
        self.CALIBRATED_DISTANCE_LOWER = (sum / 10.0) - 20
        self.CALIBRATED_DISTANCE_UPPER = (sum / 10.0) + 20
        
        print("UPPER:", self.CALIBRATED_DISTANCE_UPPER)
        print("LOWER:", self.CALIBRATED_DISTANCE_LOWER)
    def distance_tripped(self):
        distance = self.get_distance()
        print("Object at", self.distance, "cm")
        
        if (distance > self.CALIBRATED_DISTANCE_UPPER or distance < self.CALIBRATED_DISTANCE_LOWER):
            return True
        else:
            return False
        

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
            
class Alarm():
    
    def __init__ (self, buzzer_pin):
        self.buzzer = PWM(Pin(buzzer_pin))
        self.led = RGBLED()
        self.on = False
        
    def sound_alarm(self):
        self.on = True
        self.alarm_settings()
    
    def alarm_settings(self):
            self.buzzer.duty_u16(32767)
            self.buzzer.freq(1000)
            self.led.pixels_fill((255,0,0))
            self.led.pixels_show()
            utime.sleep(0.5)
            self.buzzer.freq(500)
            self.led.pixels_fill((0,0,255))
            self.led.pixels_show()
            utime.sleep(0.5)
            self.buzzer.duty_u16(0)
            self.led.pixels_fill((0,0,0))
            self.led.pixels_show()
    
    def stop_alarm(self):
        self.on = False
        self.led.pixels_fill((0,0,0))
        self.led.pixels_show()
        self.buzzer.duty_u16(0)
    
    def is_on(self):
        return self.on

    def armed_sound(self):
        for i in range(0,4):
            self.buzzer.duty_u16(32767)
            self.buzzer.freq(1000)
            utime.sleep(0.5)
            self.buzzer.duty_u16(0)
            utime.sleep(0.5)
            
    def disarmed_sound(self):
        for i in range(0,4):
            self.buzzer.duty_u16(32767)
            self.buzzer.freq(500)
            utime.sleep(0.5)
            self.buzzer.duty_u16(0)
            utime.sleep(0.5)
        

class ArmedLight():
    
    def __init__ (self, led_pin):
        self.led = Pin(led_pin, Pin.OUT)
        self.armed = False
        
    def turn_on(self):
        self.led.on()
    
    def turn_off(self):
        self.led.off()
    
    def show_calibrated(self):
        for i in range(0,8):
            self.led.on()
            utime.sleep(0.25)
            self.led.off()
            utime.sleep(0.25)
    
    def show_disarmed(self):
        self.armed = False
        self.led.off()
        
    def show_armed(self):
        self.armed = True
        for i in range(0,4):
            self.led.on()
            utime.sleep(1)
            self.led.off()
            utime.sleep(1)
        self.led.on()
            
    def is_armed(self):
        return self.armed

        

#Declaration
notification = WiFi()
rfid = RFID()
alarm = Alarm(20)
distance_sensor = DistanceSensor(18, 19)
light_sensor = LightSensor(27)
armed_light = ArmedLight(7)

def check_sensors():
    print("Object at",distance_sensor.get_distance() , "cm")
    print("Light reading:", light_sensor.get_lux())

    if distance_sensor.distance_tripped() or light_sensor.light_is_on():
        alarm.sound_alarm()
    else:
        alarm.stop_alarm()


alarm.stop_alarm()
armed_light.show_calibrated()
utime.sleep(1)
armed_light.show_disarmed()
alarm.disarmed_sound()

alarm_loop_counter = 1

while True:
    if rfid.scan_card():
        notification.connect()
        armed_light.show_armed()
        alarm.armed_sound()
        while True:
            if alarm.is_on():
                alarm.sound_alarm()
                alarm_loop_counter = alarm_loop_counter + 1
                if alarm_loop_counter % 10 == 0:
                    notification.send_alert()
                    
                if rfid.scan_card():
                    alarm.stop_alarm()
                    notification.disconnect()
                    armed_light.show_disarmed()
                    alarm.disarmed_sound()
                    alarm_loop_counter = 0
                    utime.sleep(2)
            else:
                if armed_light.is_armed():
                    if rfid.scan_card():
                        alarm.stop_alarm()
                        notification.disconnect()
                        armed_light.show_disarmed()
                        alarm.disarmed_sound()
                        utime.sleep(2)
                        continue
                    utime.sleep(0.25)
                    check_sensors()
                    utime.sleep(0.25)
                else:
                    if rfid.scan_card():
                        distance_sensor.calibrate_distance()
                        light_sensor.calibrate_light()
                        armed_light.show_calibrated()
                        notification.connect()
                        armed_light.show_armed()
                        alarm.armed_sound()
                        utime.sleep(2)
