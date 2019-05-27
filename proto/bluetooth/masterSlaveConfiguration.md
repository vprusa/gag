w# Configuration of HC05(master) connected to PC over Serial to USB converter to HC05(slave) on glove

cutecom

https://www.instructables.com/id/How-to-Configure-HC-05-Bluetooth-Module-As-Master-/

http://denethor.wlu.ca/arduino/MLT-BT05-AT-commands-TRANSLATED.pdf

https://stackoverflow.com/questions/4702216/controlling-a-usb-power-supply-on-off-with-linux

as root

echo suspend > /sys/bus/usb/devices/usb1/power/level

echo on > /sys/bus/usb/devices/usb1/power/level

echo "0" > "/sys/bus/usb/devices/usb1/power/autosuspend_delay_ms"
echo "auto" > "/sys/bus/usb/devices/usb1/power/control"

echo disabled > /sys/bus/usb/devices/usb1/power/wakeup
echo suspend > /sys/bus/usb/devices/usb1/power/level  # turn off

https://img.banggood.com/file/products/20150104013200BLE-CC41-A_AT%20Command.pdf

## Slave/Master

AT+CLRBAND
AT+ROLE0
AT+LADDR
##+LADDR=54:4A:16:08:D7:41
+LADDR=54:4A:16:08:D7:41
94:E3:6D:9B:8B:C9

#AT+UART38400,0,0
AT+BAUD6

## Master/Slave

AT+CLRBAND
AT+ROLE1
#AT+CMODE0
#AT+BIND=xxxxx
#AT+BAND544A1608D741
AT+BAND94E36D9B8BC9

#+LADDR=94:E3:6D:9B:8B:C9


# Left

AT+NAMELeftToRight
OK
AT+LADDR
74:DA:EA:8E:02:2A
AT+BAND606405A7331C

# Right

AT+NAMERightHand
AT+ROLE0
+LADDR=60:64:05:A7:33:1C
AT+LADDR
AT+BAUD6
AT+RESET
AT+BAND74DAEA8E022A


# RightPC
AT+NAMERightPC
AT+ROLE0
+LADDR=9C:1D:58:19:35:6E
AT+LADDR
AT+BAUD7
AT+RESET
AT+BAND606405A7331C


# PC Master to right

https://www.saibatudomt.com.br/2018/01/conectando-3-dispositivos-arduino-utilizando-o-modulo-bluetooth-hc-05.html

AT+NAMEPCToLeftAndRight
OK
AT+LADDR
74:DA:EA:8E:02:2A
AT+BAND606405A7331C





## RIGHT With ESP32 to PC

### USB
+NAME=Test
+ROLE=0
+LADDR=54:4A:16:08:D7:41

### ESP
AT+BAND
AT+BAND544A1608D741

# ver 3

# left pro mini Master
AT+ADDR?
OK+ADDR:D43639D84C84
AT+TYPE2
AT+MODE0
AT+ROLE1
AT+PASS468531
AT+BAUD4
AT+NAMEGAGLM


# right slave to left master
AT+NAMEGAGRS
AT+BAUD4
AT+TYPE2
AT+PASS468531
AT+MODE0
AT+ROLE0
AT+CO0D43639D84C84
AT+COND43639D84C84

# esp32 right slave to left master info

OK+ADDR:D43639BC17C6


# right hand slave at PC to "GAGGM"
AT+NAMEGAGRHPC
AT+BAUD4
ADDR:
30:AE:A4:FF:1B:A2
30AEA4FF1BA2

AT+CON30AEA4FF1BA2
AT+BAND30AEA4FF1BA2

30AEA4FF1BA2
2AB1FF4AEA03
AT+CON2AB1FF4AEA03
AT+BAND2AB1FF4AEA03
