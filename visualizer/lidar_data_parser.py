import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

from serial import Serial

# 47 = 1(Start) + 1(Datalen) + 2(Speed) + 2(StartAngle) + 36(12 * 3 DataByte) + 2(EndAngle) + 2(TimeStamp) + 1(CRC)
LIDAR_SERIAL_PACKET_SIZE = 47
LIDAR_DATA_PACKET_SIZE = 12

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

def parse_lidar_ld06(data: str) -> dict:
    """
    Parse data from the LD06 Lidar sensor
    a packet starts with 0x54 0x2C and is 47 bytes long

    returns a dictionary with the points
    """

    serialBuffer = data.split(" ")
    # serialBuffer = [int(i, 16) for i in serialBuffer_str]
    # serialBuffer_float = [
    #     struct.unpack("!f", bytes.fromhex(i))[0] for i in serialBuffer_str
    # ]
    lidarPacket = {}  # Initialize the dictionary if it doesn't exist yet

    # lidarPacket["header"] = serialBuffer[0]
    lidarPacket["dataLength"] = 0x1F & int(serialBuffer[1], 16)
    lidarPacket["radarSpeed"] = serialBuffer[3] << 8 | serialBuffer[2]
    lidarPacket["timestamp"] = serialBuffer[45] << 8 | serialBuffer[44]
    lidarPacket["crcCheck"] = serialBuffer[46]

    startAngle = serialBuffer[5] << 8 | serialBuffer[4]
    endAngle = serialBuffer[43] << 8 | serialBuffer[42]

    print(f"start anglee {startAngle}")
    # fix angle step to negative if we cross 0° during scan
    # which means first angle is bigger than the last one
    # else positive
    if endAngle > startAngle:
        modulo = 0
    else:
        modulo = 360 * 100
    angleStep: float = (endAngle + (modulo - startAngle)) / (
        lidarPacket["dataLength"] - 1
    )

    points: List[dict] = []
    # compute lidar result with previously defined angle step
    for i in range(lidarPacket["dataLength"]):
        rawDeg = startAngle + i * angleStep
        angle = 360 * 100 - (rawDeg if rawDeg <= 360 * 100 else rawDeg - 360 * 100)
        distance = int(serialBuffer[8 + i * 3 - 1] << 8 | serialBuffer[8 + i * 3 - 2])
        # Raw angles are inverted
        points.append(
            {
                "angle": angle,
                "confidence": serialBuffer[8 + i * 3],
                "distance": distance,
            }
        )

    lidarPacket["points"] = points
    return lidarPacket


def read_packet(ld06: Serial) -> Tuple[float, float]:
    """
    Read a packet from the LD06 Lidar sensor
    """
    data = ld06.readline().decode("utf-8")
    return data


def polar_to_cartesian(angle, distance):
    """
    Convert polar coordinates to cartesian.
    Angle is expected in degrees.
    """
    angle_rad = math.radians(angle)  # Convert angle from degrees to radians
    x = distance * math.cos(angle_rad)
    y = distance * math.sin(angle_rad)
    return x, y


def robotref_to_fieldref(point, robot_pos):
    """convert cartesian coordinates from the robot referential to current field referential
    in order to draw the point in processing
    """
    x, y = point
    robot_x, robot_y = robot_pos

    x_field = x + robot_x
    y_field = y + robot_y

    return x_field, y_field


def print_point(point: dict):
    print(f"dist: {point['distance']}; angle: {point['angle']}")


def print_points_in_packet(packet: dict):
    for point in packet["points"]:
        print_point(point)


def print_lidar_packet(lidarPacket: dict):
    print("Packet")
    for i in lidarPacket:
        print(lidarPacket[i])


def main(file: Path):
    with open(file, "r") as f:
        lines = f.readlines()
    for line in lines:
        packet = parse_lidar_ld06(line)
        for point in packet["points"]:
            point = robotref_to_fieldref(
                polar_to_cartesian(point["angle"], point["distance"]), (1000, 1000)
            )
            # print(point)


if __name__ == "__main__":
    main(data_file)
