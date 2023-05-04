This folder contains code to simulate LD06 on serial

prerequisites: `python3 -m pip install -r requirements.txt`
usage: Run the software and plug serial to your PC, then `./ld06_simuator.py`

Data must be formatted as a packet par line like this
```
54 2C 10 0E E4 21 F2 01 EB FA 01 EE FA 01 EE FA 01 EF FA 01 F0 F2 01 ED FA 01 EC FA 01 ED FA 01 EB F2 01 EC FA 01 EC 0A 02 EC 4C 25 BC 74 79
54 2C 10 0E 9B 25 FA 01 EE FA 01 ED 0A 02 EB 0A 02 EE FA 01 EE FA 01 F0 0A 02 ED 0A 02 EE 0A 02 EF 0A 02 EF 19 02 EE 19 02 F0 04 29 BE 74 70
54 2C 10 0E 50 29 19 02 EE 29 02 ED 29 02 EB 29 02 EB 29 02 EB 29 02 EC 29 02 EB 19 02 E5 29 02 D5 29 02 D6 29 02 D8 29 02 D8 AB 2C C1 74 AF
```
