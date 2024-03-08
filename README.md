<h1 align="center">Mecapi ESP32</h1>

## Utilisation

Get data from a LD06 LIDAR and analyze / compute points to get resulting adversary robots positions
Robots positions are then send to PIC via serial.
This code is destined to be executed on a ESP32.

If you want to plot data we advise to go on https://teleplot.fr/ instead of using vscode extension as it is bugged

## Cloner le dépôt avec les dernières version des sous-modules
```
git submodule update --init --recursive
git submodule foreach --recursive git fetch
git submodule foreach git checkout  main
```

## Interface de connection :

<details>
<summary>AZ-Delivery Esp32 DevKit C V4</summary>
<img src="wiki/AZ-Delivery ESP-32 DevKit C V4.png">
</details>
<details>
<summary>AZ-Delivery Esp32 DevKit C V4 Pinout</summary>
<img src="wiki/AZ-Delivery ESP-32 DevKit C V4 pinout.png">
</details>
<br>

| Numero | Signal          | Nom             | Description                             | Module connecté |
|--------|-----------------|-----------------|-----------------------------------------|-----------------|
Debug
|   1    | GPIO  1         | U0 TX           | Serial interface 0 transmit             | PC Serial RX    |
|   3    | GPIO  3         | U0 RX           | Serial interface 0 receive              | PC Serial TX    |
Robot
|   2    | GPIO  2         | U1 RX           | Serial interface 1 receive              | Robot Serial TX |
|   4    | GPIO  4         | U1 TX           | Serial interface 1 transmit             | Robot Serial RX |
Lidar
|  16    | GPIO 16         | U2 RX           | Serial interface 2 receive              | Lidar Serial TX |
|  17    | GPIO 17         | U2 TX           | Serial interface 2 transmit             | Not used !      |
|  23    | GPIO 23         | LD06 PWM        | PWM control for lidar motor             | Lidar PWM       |

## Les modules disponibles sont :
- Lidar LD06
- Camera TOF 3D MetaSenseA010
- Robot Mecapi

## Commandes
Via le serial 0 (USB), on peut envoyer des commandes comme suit : <br>
Une commande en ``string`` suivie de ``:`` puis des données séparées par ``;`` avec à la fin ``\n`` <br>
Exemple : ``cmdTest:0;1.0``

|Commande|Type des données|Valeurs possibles|Description|
|-|:-:|:-|-|
|PrintEnable | ``int`` | 0 = Disable Print <br> 1 = Enable Print | Permet d'activer le log sur le serial 0
|PrintLevel | ``int`` | 0 = LEVEL_VERBOSE <br> 1 = LEVEL_INFO <br> 2 = LEVEL_WARN <br> 3 = LEVEL_ERROR <br> 4 = LEVEL_NONE| Permet de choisir le niveau de log et donc d'affichage des messages
|DebugEnable | ``int`` | 0 = Disable Debug <br> 1 = Enable Debug | Permet d'activer le mode pas à pas
|DebugSteps | ``int`` | Nombre de pas | Permet d'ajouter un nombre de pas au mode pas à pas
|LD06PWM | ``int`` | 20 < PWM < 50 | Permet d'ajuster la pwm du moteur de rotation du lidar, min=20, max=50
|RobotXYA | ``int;int;int`` | 0 < x < 3000 <br> 0 < y < 3000 <br>  0 < θ < 360 | Permet de modifier la position cartésienne du robot
|RobotState | To Be Defined
|A010 | To Be Defined
|VL53 | To Be Defined
