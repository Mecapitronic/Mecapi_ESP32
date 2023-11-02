#!/usr/bin/env python3

import argparse
import time
from pathlib import Path
from sys import platform
from typing import List

from serial import Serial
import serial
import socket

teleplot = ("teleplot.fr",64390)
loopback = ("127.0.0.1",4000)
sockUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
sockTCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)

FILE = "data_ld06_3.txt"

BAUDRATE = 230400
SERIAL_LNX = "/dev/ttyUSB"
SERIAL_WIN = "COM"


def init_serial(serial_if: str) -> None:
    #if serial_if.startswith("rfc2217://"):
    ser = serial.serial_for_url(serial_if, BAUDRATE)
    #else:
    #    ser = serial.Serial(serial_if, BAUDRATE)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.close()


def send_packet(packet: bytes, serial_if: str) -> None:
    #if serial_if.startswith("rfc2217://"):
    ser = serial.serial_for_url(serial_if, BAUDRATE)
    #else:
    #    ser = serial.Serial(serial_if, BAUDRATE)
    ser.write(packet)
    ser.close()


def get_packet(serial_if: str) -> bytes:
    #if serial_if.startswith("rfc2217://"):
    ser = serial.serial_for_url(serial_if, BAUDRATE)
    #else:
    #    ser = serial.Serial(serial_if, BAUDRATE)
    packet = ser.read()
    ser.close()
    return packet


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
    print(f"Starting Sensor, sending packets to {serial_if}:")
    data_file = Path(__file__).parent / file
    #init_serial(serial_if)
    sockTCP.connect(loopback)
    sockTCP.settimeout(0.1)
    packets = packets_from_file(Path(data_file))
    for packet in packets:
        #send_packet(bytes.fromhex(packet), serial_if)
        send = sockTCP.send(bytes.fromhex(packet))
        #try:
        #    bytesRecv = sockTCP.recv(100)
        #except socket.timeout:
        #    error = 0
        #if ser.in_waiting>0:
            #with serial.Serial("\\\\.\\CNCA0", 115200) as serTeleplot:
            #    serTeleplot.write(ser.read_all())
            #    serTeleplot.close()
                #sockUDP.sendto(ser.read_all(), teleplot)
        print(f"{packets.index(packet)}/{len(packets)}", end="\r")
        time.sleep(1.1)

    print("All data sent")


if __name__ == "__main__":
    argParser = argparse.ArgumentParser()
    argParser.add_argument(
        "interface_type",
        type=str,
        default='serial',
        nargs="?",
        help="type of interface, serial or socket",
    )
    argParser.add_argument(
        "interface",
        type=str,
        default='COM5',
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
