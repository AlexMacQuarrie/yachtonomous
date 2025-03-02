# Source Code
This directory contains MicroPython source code for the boat that runs on the MCU, and Python source code than runs on a PC
- yachtonomous.py is the PC entrypoint, which runs on the PC, configured by config.json (Python)
- micropy/main.py is the MCU entrypoint, which runs automattically on the MCU (MicroPython)
- esp32 folder has code for the ESP32 Bluetooth sensor server and clients (C++)
- results/parse_csv.py is a script to parse CSV logs generated during testing and plot the states and inputs (Python)