# LWM2M example for STM32L4+ Discovery kit IoT node

For using this sample you need one: [B-L4S5I-IOT01A](https://www.st.com/en/evaluation-tools/b-l4s5i-iot01a.html)

The sample connects to a WiFi network (see in `main.c` to change both SSID and password).
And connect to EdgeIQ using the Zephyr LWM2M client.

This sample exhibits some LWM2M objects for exposing some of the device capabilities like LED control, user button, temperature and humidity sensors.

You must tweak the server URL, the credential and the endpoint in the code, based on your needs.

Use:

- checkout the code as a subdirectory of your `zephyrproject` installation
- in the application directory use `west build -b b_l4s5i_iot01a` to build the code.
- then `west flash` to program the dev kit over USB STLINK.
- connect to the serial port (`/dev/ttyACM0` on Linux) baud rate 115200 to get access to the console

If flashing works but the device behavior doesn't change, try erasing the device flash memory with ST IDE.
