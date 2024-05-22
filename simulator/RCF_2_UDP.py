#!/usr/bin/env python3

import argparse
import time
from pathlib import Path
from sys import platform
from typing import List

from serial import Serial
import serial
import socket

teleplotLocal = ("localhost",47269)
sockUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
sockTCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
serialPort = None

BAUDRATE = 921600
SERIAL_LNX = "/dev/ttyUSB"
SERIAL_WIN = "COM"


def init_serial(serial_if: str) -> None:
    global serialPort
    try:
        if serial_if.startswith("rfc2217://"):            
            serialPort = serial.serial_for_url(serial_if, BAUDRATE)
        else:
            serialPort = serial.Serial(serial_if, BAUDRATE)
        serialPort.reset_input_buffer()
        serialPort.reset_output_buffer()
    except :
        print(f"Error with Serial")
        quit()


def send_packet(packet: bytes) -> None:
    global serialPort
    serialPort.write(packet)

def packet_available() -> bool:
    global serialPort
    if serialPort.in_waiting > 0:
        return True
    return False

def get_packet() -> bytes:
    global serialPort
    if serialPort.in_waiting > 0:
        packet = serialPort.readline()
        return packet.replace(b'\r\n',b'\n')
    return None


def which_serial(serial_if: str = None, serial_num: int = 0) -> str:
    if serial_if:
        return serial_if

    if os_is_unix():
        return SERIAL_LNX + str(serial_num)
    else:
        return SERIAL_WIN + str(serial_num)

def os_is_unix() -> bool:
    if platform == "linux" or platform == "linux2" or platform == "darwin":
        return True
    elif platform == "win32":
        return False


def packets_from_file(data_file: Path) -> List:
    with open(data_file, "r") as f:
        data = f.readlines()
    return data


def main(serial_if: str):
    print(f"Starting listening -> {serial_if}")
    init_serial(serial_if)
    while True:
        if packet_available():
            packet = get_packet()
            decoded_string = packet.decode("utf-8")
            if decoded_string.startswith(">"):
                decoded_string = decoded_string.replace(">","",1)
            else:
                decoded_string = decoded_string.replace("\n","|np\n")
            sockUDP.sendto(decoded_string.encode(), teleplotLocal)

if __name__ == "__main__":
    argParser = argparse.ArgumentParser()
    argParser.add_argument(
        "interface_type",
        type=str,
        default='socket',
        nargs="?",
        help="type of interface, serial or socket",
    )
    argParser.add_argument(
        "interface",
        type=str,
        default='rfc2217://localhost:4000',
        nargs="?",
        help="interface where to send packets",
    )
    args = argParser.parse_args()

    serial_if = which_serial(serial_if=args.interface)
    main(serial_if)
