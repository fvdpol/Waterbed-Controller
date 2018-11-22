# Waterbed Controller

Note: Work in progress

# Objective

Temperature controller for waterbed heater. 



* Temperature controller on ATMega328P (Arduino)
* Dallas D18B20 Temperature sensors
* Brett Beauregard's PID Controller Library
* Output modulation using solid state relay
* Remote accessible using ESP8266 / ESPLink
* Python script to connect the controller to MQTT
* OpenHAB integration (via MQTT)


To Do:
* Fail-Safe relay in series with SSR (as failure mode is typically to fail short)
* Bypass relay in parallel with SSR to minimise dissipation if maximum output is required for extend time (eg. heating up cold bed)
* Add Display/UI