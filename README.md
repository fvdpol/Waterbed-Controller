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



# Original Controller

The original waterbed controller was properly maintaining a constant temperature. Inspection of thermostat's internals showed that the controller was purely electromechanical, so no surprise no accurate & stable regulation could be obtained.





# ESP Link Configuration

Pin Assignment
Reset - gpio3/RX0
ISP/Flash - gpio1/TX0
TX Enable - disabled
Conn LED - gpio0
Serial LED - gpio14
UART Pins - swapped
RX Pull-up - disabled

Serial Baud 57600
