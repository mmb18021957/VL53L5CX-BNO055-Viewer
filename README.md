# VL53L5CX‑BNO055 Viewer (Remote)
[![Video ansehen](assets/video_thumbnail.jpg)](https://youtu.be/zqquU1z1eHg)

### Ein echtzeitfähiger 3D‑Viewer für den **VL53L5CX Time‑of‑Flight‑Sensor** in Kombination mit einem **BNO055 IMU‑Sensor**.  
### Die Anwendung ist auf der Esp32 Seite in Micropython und auf der PC-Seite in Python programmiert.
### Als Entwicklungsumgebung wird Thonny verwendet.

### Verbindungsarten:

### 1. Nur Esp32-Wroom mit USB-C Kabel

### 2. Remote: ESP32-Wroom -->  Esp32-C3 (USB-C)

### 3. Remote: ESP32-Wroom über Wifi-AP ssid : VL53L5CX-BNO055   
<img width="987" height="772" alt="graph_Bno055_1" src="https://github.com/user-attachments/assets/8ab10044-d533-45ec-863b-7a3aa2afff66" />
<img width="1149" height="818" alt="image" src="https://github.com/user-attachments/assets/f8126538-c561-4939-b8af-7dd106eae732" />

### Hardware Verdrahtung mit ESP32-Wroom 
<img width="817" height="588" alt="i2c-vl53l5cx-bno055-1_orig" src="https://github.com/user-attachments/assets/8667779a-87de-4db9-ade0-b7779d77862b" />

### BMo055 adafruid    adr: 0x28  p_arr = [q[0],-q[2],q[1],q[3]]


### BMo055 china clone adr: 0x29  p_arr = [q[0],-q[1],-q[2],q[3]]
<img width="698" height="336" alt="bno055-china" src="https://github.com/user-attachments/assets/167e1672-32f7-4d84-b46c-a06094be4d2a" />

## Software:

### mp-extras/vl53l5cx https://github.com/mp-extras/vl53l5cx

### main_data_wifi.py :  esp32 wroom
### create "lib" and upload in "lib" directory : ​vl53l5cx
### copy into root folder : sensors.py
<img width="734" height="656" alt="vl53l5cx-libs-sensor_orig" src="https://github.com/user-attachments/assets/e2c05c2a-552f-4d05-96c9-99ce7bf6b712" />

### lib BNo055 https://github.com/micropython-IMU/micropython-bno055
### copy into root folder : bno055.py, bno055_base,py
<img width="346" height="356" alt="mpBno055" src="https://github.com/user-attachments/assets/57a64eb0-7e7d-4af1-ab01-9f07da18e1a6" />

### Funktionsweise und optischer Aufbau des Senders: 
### Das System sendet Infrarotlicht (940 nm VCSEL, Vertical-Cavity Surface-Emitting Laser) aus. Ein diffraktives optisches Element (DOE) beinflusst den Strahl so, dass ein quadratisches Sichtfeld (Field of View, FoV) von bis zu 63° ausgeleuchtet wird.
### Empfänger: 
### Das reflektierte Licht wird über ein Empfänger-Linsensystem auf eine spezielle Empfangsmatrix geleitet. Diese besteht aus einer SPAD-Anordnung (Single Photon Avalanche Diode), die in 64 einzelne Messzonen (8 x 8 Gitter) unterteilt ist.
### Distanzberechnung: 
### Der Sensor nutzt die Direct ToF-Technologie. Er misst direkt die Zeit \(t\), die ein Laserpuls für die Strecke vom Sensor zum Objekt und wieder zurück benötigt. Die Entfernung \(s\) zum Objekt wird anschließend anhand der konstanten Lichtgeschwindigkeit \(c\) berechnet. Dies wird für alle Strahlen (64) berechnet. Deshalb ergibt sich für ein 4x4 FoV eine Frequenz von 60Hz und für ein 8x8 FoV eine Frequenz von 15 Hz.

### https://www.glowscript.org/#/user/mmb18/folder/MyPrograms/program/VL53L5CX-BNO055-viewer
<img width="388" height="390" alt="VpythonWeb-FoV-ToF" src="https://github.com/user-attachments/assets/53c8d1f5-9339-4a92-a331-a29f39d62dcc" />

### Repository klonen

git clone https://github.com/mmb18021957/VL53L5CX-BNO055-Viewer.git

cd VL53L5CX-BNO055-Viewer
"# VL53L5CX-BNO055-Viewer" 
