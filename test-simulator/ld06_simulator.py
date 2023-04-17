#!/usr/bin/env python3

import serial
import time
from pathlib import Path
from typing import List

FILE="data_ld06.txt"


def send_packet(packet: bytes):
    ser = serial.Serial('/dev/ttyUSB0', 230400)
    ser.write(packet)
    ser.close() 


def packets_from_file(data_file: Path) -> List:
    with open(data_file, 'r') as f:
        data = f.readlines()
    return data

def convert_packet(packet_str: str) -> bytes:
    packet_str = packet_str.replace('0x', '')
    return bytes.fromhex(packet_str) 

def main():
    print("Starting Lidar, sending packets:")
    data_file = Path(__file__).parent / FILE

    lidar_packets = packets_from_file(Path(data_file))
    for packet in lidar_packets:
        send_packet(convert_packet(packet))

        print(f"{lidar_packets.index(packet)}/{len(lidar_packets)}", end="\r")
        time.sleep(0.10)

    print("All data sent")


if __name__ == "__main__":
    main()