from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

from serial import Serial

DATA_FOLDER = Path(__file__).parent.parent / "simulator/LD06"

# data_files = DATA_FOLDER.iterdir()
data_file = DATA_FOLDER / "data_ld06_1.txt"


def read_file(file: Path) -> List[str]:
    with open(file, 'r') as f:
        data = f.readlines()

    return data

@dataclass
class LidarPacket:
    header: str = "0x56 0x2C"

def parse_lidar_ld06(data: str) -> Tuple[float, float]:
    """
    Parse data from the LD06 Lidar sensor
    a packet starts with 0x54 0x2C and is 47 bytes long

    lidarPacket.header = serialBuffer[0];
    lidarPacket.dataLength = 0x1F & serialBuffer[1];
    lidarPacket.radarSpeed = serialBuffer[3] << 8 | serialBuffer[2];
    lidarPacket.startAngle = serialBuffer[5] << 8 | serialBuffer[4];

    lidarPacket.endAngle = serialBuffer[43] << 8 | serialBuffer[42];
    lidarPacket.timestamp = serialBuffer[45] << 8 | serialBuffer[44];
    lidarPacket.crcCheck = serialBuffer[46];


    """

    data = data.split(",")
    data = [float(x) for x in data]
    angle = data[0]
    distance = data[1]
    return angle, distance



def read_packet(ld06: Serial) -> Tuple[float, float]:
    """
    Read a packet from the LD06 Lidar sensor
    """
    data = ld06.readline().decode("utf-8")
    return data


def print_point(point: Tuple[float, float]):
    print(f"dist: {point[0]}; angle: {point[1]}")


def main(file: Path):
    with open(file, "r") as f:
        lines = f.readlines()
    for line in lines:
        point = parse_lidar_ld06(line)
        print_point(point)


if __name__ == "__main__":
    main(data_file)
