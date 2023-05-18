from machine import Pin
from mfrc522 import MFRC522
import utime

reader = MFRC522(spi_id=0, sck=2, miso=4, mosi=3, cs=1, rst=0)
led = Pin(25, Pin.OUT)

print("")
print("Place card into reader")
print("")

PreviousCard = [0]
registered_uids = [[0x03, 0x5A, 0x0C, 0x0E]]


try:
    while True:
        reader.init()
        (stat, tag_type) = reader.request(reader.REQIDL)
        if stat == reader.OK:
            (stat, uid) = reader.SelectTagSN()
            if uid == PreviousCard:
                continue

            if stat == reader.OK:
                print("Card detected {}  uid={}".format(hex(int.from_bytes(bytes(uid), "little", False)).upper(), reader.tohexstring(uid)))

                if uid in registered_uids:
                    print("Registered card!")
                    led.value(1)
                    utime.sleep_ms(1000)
                    led.value(0)
                else:
                    print("Unregistered card!!")

                PreviousCard = uid
        else:
            PreviousCard = [0]
        utime.sleep_ms(50)

except KeyboardInterrupt:
    print("Bye")
