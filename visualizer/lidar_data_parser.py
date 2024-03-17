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

    serialBuffer_str = data.split(" ")
    serialBuffer = [int(i, 16) for i in serialBuffer_str]
    # serialBuffer_float = [
    #     struct.unpack("!f", bytes.fromhex(i))[0] for i in serialBuffer_str
    # ]
    lidarPacket = {}  # Initialize the dictionary if it doesn't exist yet

    lidarPacket["header"] = serialBuffer[0]
    lidarPacket["dataLength"] = 0x1F & serialBuffer[1]
    lidarPacket["radarSpeed"] = serialBuffer[3] << 8 | serialBuffer[2]
    lidarPacket["startAngle"] = serialBuffer[5] << 8 | serialBuffer[4]

    lidarPacket["endAngle"] = serialBuffer[43] << 8 | serialBuffer[42]
    lidarPacket["timestamp"] = serialBuffer[45] << 8 | serialBuffer[44]
    lidarPacket["crcCheck"] = serialBuffer[46]

    angleStep = 0
    # fix angle step to negative if we cross 0° during scan
    # which means first angle is bigger than the last one
    # else positive
    if lidarPacket["endAngle"] > lidarPacket["startAngle"]:
        angleStep = (lidarPacket["endAngle"] - lidarPacket["startAngle"]) / (
            lidarPacket["dataLength"] - 1
        )
    else:
        angleStep = (
            lidarPacket["endAngle"] + (360 * 100 - lidarPacket["startAngle"])
        ) / (lidarPacket["dataLength"] - 1)

    # compute lidar result with previously defined angle step
    for i in range(LIDAR_DATA_PACKET_SIZE):
        rawDeg = lidarPacket["startAngle"] + i * angleStep + LIDAR_ROBOT_ANGLE_OFFSET
        # Raw angles are inverted
        lidarPacket["dataPoint"][i]["angle"] = 360 * 100 - (
            rawDeg if rawDeg <= 360 * 100 else rawDeg - 360 * 100
        )
        lidarPacket["dataPoint"][i]["confidence"] = serialBuffer[8 + i * 3]
        lidarPacket["dataPoint"][i]["distance"] = int(
            serialBuffer[8 + i * 3 - 1] << 8 | serialBuffer[8 + i * 3 - 2]
        )

    print(lidarPacket)
    # numbers = [float(x) for x in bytes]
    # angle = numbers[0]
    # distance = numbers[1]
    # return angle, distance

    return lidarPacket


def read_packet(ld06: Serial) -> Tuple[float, float]:
    """
    Read a packet from the LD06 Lidar sensor
    """
    data = ld06.readline().decode("utf-8")
    return data


def print_point(point: Tuple[float, float]):
    print(f"dist: {point[0]}; angle: {point[1]}")

def print_lidar_packet(lidarPacket: dict):
    print("Packet")
    for i in lidarPacket:
        print(f"    {lidarPacket[i]}")


def main(file: Path):
    with open(file, "r") as f:
        lines = f.readlines()
    for line in lines:
        point = parse_lidar_ld06(line)
        print_lidar_packet(point)


if __name__ == "__main__":
    main(data_file)
