from machine import Pin
from esp8266 import ESP8266

## Create an ESP8266 Object
esp01 = ESP8266()
esp8266_at_ver = None

# print("StartUP",esp01.startUP())
# print("Echo-Off",esp01.echoING())
# print("\r\n")
# 
# '''
# Print ESP8266 AT comand version and SDK details
# '''
# esp8266_at_ver = esp01.getVersion()
# if(esp8266_at_ver != None):
#     print(esp8266_at_ver)
# 
# '''
# set the current WiFi in SoftAP+STA
# '''
# print("WiFi Current Mode:",esp01.setCurrentWiFiMode())
#   
# print("\r\n\r\n")
# 
# '''
# Connect with the WiFi
# '''
# print("Try to connect with the WiFi..")
# while (1):
#     if "WIFI CONNECTED" in esp01.connectWiFi("DS Stylelit",""):
#         print("ESP8266 connect with the WiFi..")
#         break;
#     else:
#         print(".")
#         time.sleep(2)

# while(1):    
#     '''
#     Going to do HTTP Get Operation with https://maker.ifttt.com/trigger/alarm_triggered/json/with/key/b0oAtGCg63Jhlk21KfBGLiQfhXY3LQW7YDy_AQoUb0c, It return the IP address of the connected device
#     '''
#     httpCode, httpRes = esp01.doHttpGet("maker.ifttt.com","/trigger/alarm_triggered/json/with/key/b0oAtGCg63Jhlk21KfBGLiQfhXY3LQW7YDy_AQoUb0c","RaspberryPi-Pico", port=80)
#     print("------------- https://maker.ifttt.com/trigger/alarm_triggered/json/with/key/b0oAtGCg63Jhlk21KfBGLiQfhXY3LQW7YDy_AQoUb0c Get Operation Result -----------------------")
#     print("HTTP Code:",httpCode)
#     print("HTTP Response:",httpRes)
#     #print("-----------------------------------------------------------------------------\r\n\r\n")
#     esp01.disconnectWiFi()

esp01.connectWiFi("DS Stylelit","Password01")
httpCode, httpRes = esp01.doHttpGet("maker.ifttt.com","/trigger/alarm_trigger/json/with/key/yPlK3vBbmFurEZRSVqjW8S3UmsyDzaGd8wsY6Y5kfW","RaspberryPi-Pico", port=80)
esp01.disconnectWiFi()

    