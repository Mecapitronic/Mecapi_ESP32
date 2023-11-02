# SerialPacketSimulator

Send packet over serial to your application to simulate a sensor

## Prerequisites:

`python3 -m pip install -r requirements.txt`

## Usage:

Plug your serial from your PC to your application  
Choose which sensor to simulate, then execute :

`./Serial_Packet_Simulator.py COM6 ld06 2`

-   First argument is the serial port : "/dev/ttyUSB" or COM6, or rfc2217://localhost:4000, or \\\\.\\CNCA0
-   Second argument is the type of sensor, for now : ld06 or a010
-   Third argument is the number of sample file to send, example : data_ld06_1.txt
    <br />
    (See config at end for VSCode Debug)<br />

Data must be formatted as a packet par line like this

### Lidar TOF LD06

```
54 2C 10 0E E4 21 F2 01 EB FA 01 EE FA 01 EE FA 01 EF FA 01 F0 F2 01 ED FA 01 EC FA 01 ED FA 01 EB F2 01 EC FA 01 EC 0A 02 EC 4C 25 BC 74 79
```

<br />

```
54 2C 10 0E 9B 25 FA 01 EE FA 01 ED 0A 02 EB 0A 02 EE FA 01 EE FA 01 F0 0A 02 ED 0A 02 EE 0A 02 EF 0A 02 EF 19 02 EE 19 02 F0 04 29 BE 74 70
```

<br />

```
54 2C 10 0E 50 29 19 02 EE 29 02 ED 29 02 EB 29 02 EB 29 02 EB 29 02 EC 29 02 EB 19 02 E5 29 02 D5 29 02 D6 29 02 D8 29 02 D8 AB 2C C1 74 AF
```

### Camera TOD A010

```
00 FF DD
```

## VS Code Debug Config at /.vscode/launch.json

```{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Python : fichier actif",
            "type": "python",
            "request": "launch",
            "program": "${file}",
            "console": "integratedTerminal",
            "justMyCode": true,
            "args": ["COM10", "ld06", "2"]
        }
    ]
}
```
