import tty,sys,select,termios
import serial
import struct
from time import sleep
from random import randint

uart6 = serial.Serial("/dev/ttyUSB0", 460800)
def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist = select.select([sys.stdin],[],[],0.1)
    
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
    return key
def uart_send(speed, buzzer, servo):
    xorCheck = 0
    txFrame = struct.pack("<BbHfBBBB", 0x34, buzzer, servo, speed, 0, 0, 0, 0x43)
    for i in txFrame:
        xorCheck ^= i
    txFrame = struct.pack("<BbHfBBBB", 0x34, buzzer, servo, speed, 0, 0, xorCheck, 0x43)
    uart6.write(txFrame)

direction = 1
try:
    while True:
        setting = termios.tcgetattr(sys.stdin)
        InPut = getKey(setting)
        buzzer = -1
        servo = 1500 
        speed = 2.5

        if InPut == "q":
            break
        if InPut == "w":
            speed = 1.5
            servo = 1500 
            uart_send(speed, buzzer, servo)
        if InPut == "s":
            speed = -1.5
            servo =  1500 
            uart_send(speed, buzzer, servo)
        if InPut == "d":
            speed = direction * 1.5
            servo = 1300 
            uart_send(speed, buzzer, servo)
        if InPut == "a":
            speed = direction * 1.5
            servo = 1700 
            uart_send(speed, buzzer, servo)
        if InPut == "e":
            direction *= -1
except KeyboardInterrupt:
    uart6.close()
    print(txFrame.hex())
uart6.close()
