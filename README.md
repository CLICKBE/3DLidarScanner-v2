# 3D LiDAR scanner v2 - With stepper motors, Arduino UNO and Arduino CNC Shield v3

This repository demonstrates a homemade 3D LiDAR scanner made out of a [Benewake TF-Mini LiDAR sensor](https://www.gotronic.fr/art-capteur-de-distance-lidar-tf-mini-27615.htm) and 2 stepper motors NEMA17 ([17HS4401](https://boutique.semageek.com/fr/1443-moteur-pas-a-pas-17hs4401-12v-nema17-200-pasrev-17a-3005762453528.html) and [Casun 42SHD0001-24B](http://www.all-electronics-online.com/china-828911685/42shd0001-24b-high-torque-12v-dc-nema-17-stepper-motor-for-3d-printer.html)). This scanner can produce 3D cloud point of spaces.
The LiDAR sensor is fixed on one stepper arm with the help of a 3D printed bracket. This stepper is fixed to the second stepper arm through another bracket. 


|<img height="300" src="https://github.com/CLICKBE/MWE-scanner_stepper/assets/2494294/8350b0a7-daef-4660-8843-7ecf97d9d9a3" alt="scannerLidar-stepper"> | <img height="300" src="https://github.com/CLICKBE/MWE-scanner_stepper/assets/2494294/b51f2bb5-2d3f-4970-921e-cd1419681865" alt="scan3D-exemple">|
| :---: | :---: |
| The 3D LiDAR scanner| Example of 3D visualisation of the scanning |

## Files in the repository
- src/main.cpp: file to be uploaded to the Arduino Uno board;
- src/stepperScanner-serial2CSV.py: script to save scan data into a CSV file;
- 3D_support: folder containing different 3D models for supports made; 
- fritzing: [Fritzing](https://fritzing.org/) file used to draw the connection picture.


## Elements to build the scanner

### Parts
- Benewake TF-Mini Lidar sensor: [https://www.gotronic.fr/art-capteur-de-distance-lidar-tf-mini-27615.htm](https://www.gotronic.fr/art-capteur-de-distance-lidar-tf-mini-27615.htm); 
- Arduino CNC Shield v3: [https://blog.protoneer.co.nz/arduino-cnc-shield/](https://blog.protoneer.co.nz/arduino-cnc-shield/);
- Horizontal stepper: [17HS4401](https://boutique.semageek.com/fr/1443-moteur-pas-a-pas-17hs4401-12v-nema17-200-pasrev-17a-3005762453528.html), NEMA 17, 12V, 1.7A, 200 step/rev;
- Vertical stepper: [Casun 42SHD0001-24B](http://www.all-electronics-online.com/china-828911685/42shd0001-24b-high-torque-12v-dc-nema-17-stepper-motor-for-3d-printer.html), NEMA17, 12V, 0.4A, 200 step/rev;
- External power: 12V, 2A;
- (optional) Maker Beam aluminium profiles and hardware or other.

### Connection

![3DLidarScanner with stepper connections](https://github.com/CLICKBE/3DLidarScanner-v2/blob/main/fritzing/3DLidarScanner-v2-connections.png?raw=true)

Note that this diagram replicates the use of the Arduino CNC Shield v3 by using the same pin to connect the two drivers.

### 3D models
To make this prototype we printed supports (that you can see on the picture above): one to connect the two steppers together and the other one to support the LiDAR senor and connect it to one of the NEMA 17 stepper.

If you need to edit the 3d models you can use the [Freecad](https://www.freecad.org/) files (*.FCStd).

More 3D models for steppers and LiDAR sensors can be found in a specific repo : [https://github.com/CLICKBE/3DModelsFor3dPrinters](https://github.com/CLICKBE/3DModelsFor3dPrinters).


## Using the 3D LiDAR scanner v2

### Code

This code has been developed with VSCode and PlatformIO backend. This repo therefore reproduced the folder structure used in such project. Once you opened VSCode, open PlatformIO home page and hit Open Project button. Then, select the folder of the repository.

If you don't know anything about VSCode and PlatformIO, here's a link to set you up : [https://platformio.org/install/ide?install=vscode](https://platformio.org/install/ide?install=vscode).

### Library needed : 
- TFMPlus: [https://github.com/budryerson/TFMini-Plus](https://github.com/budryerson/TFMini-Plus). The library is not included in this git repository but is referenced in the [platformio.ini](https://github.com/CLICKBE/3DLidarScanner-v2/blob/main/platformio.ini) file and therefore should directly be downloaded by PlatformIO extension of VSCode when building the code.
- SoftwareSerial: included in Arduino framework

### Scanner serial protocol
Once the Arduino script is uploaded and running onto the Arduino Uno board, the scanner can be control through serial protocol at 115200 bauds. The following commands are available : 

- s : perform 2D scan;
- y : perform 1D scan (vertical);
- i : display scanner setup info;
- v : followed by an integer value sets the vertical step (e.g. `v 1`);
- h : followed by an integer value sets the horizontal step (e.g. `h 1`);
- p : scan reboot.
   
Once the scan is launched (through s or y), it performs the XYZ coordinates conversion and ouputs all of the data through serial port in the following manner : 

`x y z h_idx v_idx distance`

With : 
- x: x coordinate in 3D space;
- y: y coordinate in 3D space;
- z: z coordinate in 3D space;
- h_idx: horizontal index of the scan;
- v_idx: vertical index of the scan;
- distance: data coming from TF-Mini-s LiDAR sensor.


### Parsing data to CSV files
The `stepperScanner-serial2CSV.py` python script establishes serial communication with the Arduino Uno used for the 3D Lidar scanner. Once the script is launched, it stores incoming data (`x y z h_idx v_idx distance`) into a csv file.

Usage is as follows (one can use `python stepperScaneer-serial2CSV.py -h` to check on the available option arguments) : 

`python stepperScanner-serial2CSV.py -p YOUR_ARDUINO_SERIAL_PORT -b 115200 -f /your/filepath/to/the/data/storage.csv`

Once the script is launched, the serial commands described above can be used to control the scanner.

## 3D Visualization of the scan

To perform a quick 3D visualization based on the serial data you can use the Processing script developed by Dana Peters : [LidarScanner.pde](https://drive.google.com/file/d/1D5wfzA8i0Pzh4qe-1skmpnqmhrvaq9d3/view?usp=drive_web) who also developed a [3D LiDAR scanner](https://www.qcontinuum.org/lidar-scanner).

In order to use this script, you will need free [Processing](https://processing.org/) software.

In order to send commands from this Processing sketch to the scanner, you can modified the `keyPressed()` function of the code by adding the following `else` condition to the `if, else if` statements that are present :  
```java
else 
{
    try 
    {
      serial.write( key );
      println( "Sending command to scanner" );
    }
    catch (Exception e)
    {
      println( "Exception " + e );
    }
}
```

## License
 © 2022 – CLICK - Université de Mons

3D LiDAR scanner v2 - with stepper motors – CLICK - UMONS (Loïc Reboursière, Maxime Vander Goten, Sami Yunus) is free software: you can redistribute it and/or modify it under the terms of the Apache License Version 2.0. This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the Apache License Version 2.0 for more details.
You should have received a copy of the Apache License Version 2.0 along with this program.  If not, see http://www.apache.org/licenses/
Each use of this software must be attributed to University of MONS – CLICK (Loïc Reboursière, Maxime Vander Goten, Sami Yunus).
Any other additional authorizations may be asked to avre@umons.ac.be.

## Legal Notices
This work was produced as part of the FEDER Digistorm project, co-financed by the European Union and the Wallonia Region.

![Logo FEDER-FSE](https://www.enmieux.be/sites/default/files/assets/media-files/signatures/vignette_FEDER%2Bwallonie.png)
