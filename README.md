# Ripple Robot
A Velox-style amphibious robot

https://github.com/user-attachments/assets/b1fe532d-10aa-4c21-8a36-f8408d190a4b

### Overview
This repository currently contains the project files for Ripple, a small amphibious robot inspired by fin-propelled Velox platforms. The design emphasizes simple manufacturing and a modular architecture.

<img width="1920" height="1440" alt="image" src="https://github.com/user-attachments/assets/746e21ad-7df4-4772-8689-78c7abd57b77" />

<img width="1440" height="1920" alt="image" src="https://github.com/user-attachments/assets/3c33fdc8-ea46-46c9-8b8e-dc29fd17e8c1" />

<img width="3072" height="4096" alt="image" src="https://github.com/user-attachments/assets/ed6f7f00-6369-414c-a1fb-26ea322d681d" />

### Contents
- ([cad](cad)) – All Inventor part files for the robot
- ([code](code)) - All of the code to run the robot

### Hardware Needed
- Raspberry Pi
- Arduino
- USB Camera
- 8 servo motors
- Buck converters (at least one to step down to 5v for pi)
- 7.8v Lipo Battery
- 3D printed chassis
- 24 M4 x 12mm (bolt + nut)
- 8 M4 x 30mm (bolt + nut)

### To get started:
On the Pi:
- ./start_video.sh
- python3 pi_bridge.py

On the mac:
- test the connection to the Pi from the terminal: ping 192.168.100.10
- uv run laptop_controller_mac.py

(Note that the Pi's IP address must be static and set to 192.168.100.10)
