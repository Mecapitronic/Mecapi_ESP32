#!/usr/bin/env python3

import argparse
import time
from pathlib import Path
from sys import platform
from typing import List

from serial import Serial
import serial
import socket

teleplot = ("teleplot.fr",51545)
teleplotLocal = ("localhost",47269)
loopback = ("127.0.0.1",4000)
sockUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
sockTCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
serialPort = None

FILE = "data_ld06_3.txt"

BAUDRATE = 230400
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


def send_packet(packet: bytes, serial_if: str) -> None:
    global serialPort
    serialPort.write(packet)


def get_packet(serial_if: str) -> bytes:
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


def main(serial_if: str, file: str):
    print(f"Starting Sensor -> {serial_if}")
    data_file = Path(__file__).parent / file
    init_serial(serial_if)
    #sockTCP.connect(loopback)
    #sockTCP.settimeout(0.1)
    packets = packets_from_file(Path(data_file))
    packetNum=0
    print(f"Sending packets...")
    for packet in packets:
        send_packet(bytes.fromhex(packet), serial_if)
        print(f"{packets.index(packet)}/{len(packets)}", end="\r")
        time.sleep(0.026)
        packetNum=packetNum+1
        # 1s / (360°/0.8°/12points) = 1s / 37.5 packets = 0,026 s/packet
        if packetNum >37:
            packetNum = 0 #put breakpoint here to send 1 complete turn
    print("All data sent")


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
    argParser.add_argument(
        "sensor_type",
        type=str,
        default='ld06',
        nargs="?",
        help="type of sensor to simulate",
    )
    argParser.add_argument(
        "file_number",
        type=str,
        default='1',
        nargs="?",
        help="number of data file to send",
    )
    args = argParser.parse_args()

    serial_if = which_serial(serial_if=args.interface)
    file = args.sensor_type + "\\data_" + args.sensor_type + "_" + str(args.file_number) + ".txt"
    main(serial_if, file)
