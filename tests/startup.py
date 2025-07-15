import serial
import struct
import subprocess
import os
import shutil
from time import sleep
from itertools import cycle

uart = serial.Serial('/dev/ttyUSB0', 460800)
files = os.listdir()
config_list = []
for filex in files:
    if filex.endswith(".json"):
        config_list.append(filex)
current_config = cycle (config_list)

def uart_send(speed, buzzer, servo):
    xorCheck = 0
    txFrame = struct.pack("<BbHfBBBB", 0x34, buzzer, servo, speed, 0, 0, 0, 0x43)
    for i in txFrame:
        xorCheck ^= i
    txFrame = struct.pack("<BbHfBBBB", 0x34, buzzer, servo, speed, 0, 0, xorCheck, 0x43)
    uart.write(txFrame)
uart_send(0, 0, 1500)
sleep(0.5)
uart_send(0, 0, 1500)
sleep(0.5)
uart_send(0, 0, 1500)

def run_icar():
    uart.close()
    with open("./icar.log", "a") as log:
        print("开启进程")
        icarp = subprocess.run(['sudo', './icar'], stdout=log, stderr=log)
        print("进程退出")

while True:
    print("等待串口消息")
    if not uart.is_open:
        uart.open()
    while uart.readable():
        try:
            data = uart.read(24)
            pack = struct.unpack("<BBBBffffBBBB", data)
            if pack[1] == 2:
               run_icar() 
            elif pack[1] == 4:
                shutil.copy(next(current_config), "./config/config.json")
                uart_send(0, 3, 1500)

        except serial.SerialException as e:
            print(f"Serial error: {e}")
            break
    