# STM32L4 Discovery IoT Node LwM2M Example with ZephyrOS

This repository contains an example demonstrating how to integrate the Lightweight M2M (LwM2M) client with Zephyr OS on an STM32L4 Discovery IoT node. The example provides a practical guide to configuring and using the built-in LwM2M client offered by Zephyr OS for IoT devices.

## Table of Contents

1. [Requirements](#Requirements)
2. [Setting up the Development Environment](#SettinguptheDevelopmentEnvironment)
3. [Configuring the LwM2M Client](#ConfiguringtheLwM2MClient)
4. [Building and Flashing the Firmware](#BuildingandFlashingtheFirmware)
5. [Testing and Observing the LwM2M Client](#TestingandObservingtheLwM2MClient)
6. [Adapting the Example for Other Platforms](#AdaptingtheExampleforOtherPlatforms)

##  1. <a name='Requirements'></a>Requirements

* STM32L4 Discovery IoT Node (documentation: https://www.st.com/en/evaluation-tools/b-l475e-iot01a.html)
* Zephyr OS-compatible development environment
* LwM2M device provisioned on the server for testing and observing the client

##  2. <a name='SettinguptheDevelopmentEnvironment'></a>Setting up the Development Environment

Follow the instructions in the Zephyr OS documentation to set up your development environment: https://docs.zephyrproject.org/latest/getting_started/index.html

Clone this repository as a subdirectory within your Zephyr project installation:

```
cd <path_to_your_zephyrproject_directory>
git clone https://github.com/EdgeIQ/b-l475e-iot01a-lwm2m-example.git
```

##  3. <a name='ConfiguringtheLwM2MClient'></a>Configuring the LwM2M Client

Open the `src/main.c` file in the project directory.

Update the `AUTO_CONNECT_SSID` macro with the name of your WiFi network and `AUTO_CONNECT_SSID_PSK` with the related password.

Set the macro `ENDPOINT` to the device's unique ID as set in EdgeIQ and set the `PSK` to the lwm2m-server-psk you set also at device creation.

Optionally you can modify other values like the manufacturer or model number reported in the device object.

##  4. <a name='BuildingandFlashingtheFirmware'></a>Building and Flashing the Firmware

* In your terminal, navigate to the example directory within your Zephyr project installation.
* Compile the code by running:
```
west build -b b_l4s5i_iot01a
```
* Flash the firmware onto your SM32L4Discovery IoT Node using:
```
west flash
```

If flashing works but the device behavior doesn't change, try erasing the device flash memory with ST IDE.

##  5. <a name='TestingandObservingtheLwM2MClient'></a>Testing and Observing the LwM2M Client

Connect to the serial port of the STM32L4 Discovery IoT Node. On Linux systems, the port is typically `/dev/ttyACM0`. You may need to adjust the port for your specific operating system.

Use a serial terminal, such as PuTTY or Minicom, to access the console. Configure the terminal with a baud rate of 115200.
Observe the LwM2M client connecting to the LwM2M server and any additional console output.

##  6. <a name='AdaptingtheExampleforOtherPlatforms'></a>Adapting the Example for Other Platforms

The example is designed for the STM32L4 Discovery IoT node, but it can be utilized as a starting point for various platforms and connectivity options. To adjust the example to accommodate different hardware or communication methods, you might need to modify configuration settings, device drivers, and hardware-specific code as needed.
