#!/usr/bin/env python3

import argparse
import serial
import time
from pathlib import Path
from typing import List
from sys import platform

FILE = "data_ld06_2.txt"
SERIAL_LNX = '/dev/ttyUSB0'
SERIAL_WIN = 'COM0'
BAUDRATE = 230400

def init_serial(serial_if: str) -> None:
    ser = serial.Serial(serial_if, BAUDRATE)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.close()

def send_packet(packet: bytes, serial_if: str) -> None:
    ser = serial.Serial(serial_if, BAUDRATE)
    ser.write(packet)
    ser.close()


def packets_from_file(data_file: Path) -> List:
    with open(data_file, 'r') as f:
        data = f.readlines()
    return data

def convert_packet(packet_str: str) -> bytes:
    #packet_str = packet_str.replace('0x', '')
    return bytes.fromhex(packet_str)

def os_is_unix() -> bool:
    if platform == "linux" or platform == "linux2" or platform == "darwin":
        return True
    elif platform == "win32":
        return False


def main(serial_if: str):
    print(f"Starting Lidar, sending packets to {serial_if}:")
    data_file = Path(__file__).parent / FILE
    init_serial(serial_if)

    lidar_packets = packets_from_file(Path(data_file))
    for packet in lidar_packets:
        send_packet(convert_packet(packet), serial_if)

        print(f"{lidar_packets.index(packet)}/{len(lidar_packets)}", end="\r")
        time.sleep(0.10)

    print("All data sent")

def which_serial_if(serial_if: str = None) -> str:
    if serial_if:
        return serial_if

    if os_is_unix():
        return SERIAL_LNX

    return SERIAL_WIN

if __name__ == "__main__":
    argParser = argparse.ArgumentParser()
    argParser.add_argument("serial_interface", type=str, default=None, nargs='?', help="serial interface where to send LIDAR packets")
    args = argParser.parse_args()

    serial_if = which_serial_if(args.serial_interface)
    main(serial_if)
