#
# read waterbed data via esp-link (telnet connection)
# outputs data to mqtt via Homie IoT framework 
#
#
# installation:
#
# pip3 install homie
#
#
# Todo:
#   update of energy counter in mcu
#   network/socket re-connection
#

import time
import socket
import select
import homie
import logging


ESP_HOST = '192.168.6.61'
ESP_PORT = 23

MQTT_HOST = '192.168.1.51'

def open_esp_connection():
    while True:
        try:
            print ("connecting to ", ESP_HOST)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5)
            s.connect(('192.168.6.61', 23))
            return s
        except socket.error as e:
            print ("socket error ", format(e))
            time.sleep(5)



#read one line from the socket
def buffered_readLine(socket):
    line = ""
    while True:
        # receive single character
        socket.settimeout(30)
        char=""
        try:
            rcv = socket.recv(1) 
            char = rcv.decode('ascii')

        except Exception as e:
            print ("socket error")
            print (e)
            # reconnect...
            # should only reconnect in case of network related errors....
            
            time.sleep(5) 
            socket = open_esp_connection()
#            if e.errno == errno.ECONNRESET:
#                # Handle disconnection -- close & reopen socket etc.
#                print ("connection reset -- try to reconnect")
#            else:
#                # Other error, re-raise
            #print ("other error on recv()")
            #raise

        if char != "\n":
            # ignore any control characters
            if (char >= " "):        
                line+=char
        elif char == "\n":
            break

    return line



# send the command to the esp
def send_command(command):
    command = command + '\n'
    esp_socket.send(command.encode('ascii'))



def EnergyUpdateInputHandler(mqttc, obj, msg):
    print ("EnergyUpdateInputHandler() called")

    payload = msg.payload.decode("UTF-8")
    print (payload)
    command = "energy " + payload
    print (command)
    send_command(command)

    #update Counter to
#    if payload == 'true':
#        logger.info("Switch: ON")
#        switchNode.setProperty("on").send("true")
#    else:
#        logger.info("Switch: OFF")
#        switchNode.setProperty("on").send("false")



def main():


    while True:
        data = buffered_readLine(esp_socket)
        print ("esp: ", data)
        values = data.split(":")
        if (len(values) == 2):
            #print ("values=",values)
            field = values[0].strip()
            value = values[1].strip()

            # wrap in exception handler as sometimes the 2nd arguments is not convertable to a float/int
            try:
                if (field == "Sensor 1"):
                    fval = float(value)
                    if (fval > 10.0 and fval < 40.0):
                        NodeSensor1Temperature.setProperty("temperature").send(fval)

                elif (field == "Sensor 2"):
                    fval = float(value)
                    if (fval > 10.0 and fval < 40.0):
                        NodeSensor2Temperature.setProperty("temperature").send(fval)

                elif (field == "Sensor 3"):
                    fval = float(value)
                    if (fval > 10.0 and fval < 40.0):
                        NodeSensor3Temperature.setProperty("temperature").send(fval)

                elif (field == "PID Measured"):
                    fval = float(value)
                    if (fval > 10.0 and fval < 40.0):
                        NodePIDMeasured.setProperty("measured").send(fval)

                elif (field == "PID Setpoint"):
                    fval = float(value)
                    if (fval > 10.0 and fval < 40.0):
                        NodePIDSetpoint.setProperty("setpoint").send(fval)

                elif (field == "PID Output"):
                    fval = float(value)
                    if (fval >= 0.0 and fval <= 100.0):
                        NodePIDOutput.setProperty("output").send(fval)

                elif (field == "PID Mode"):
                    ival = int(value)
                    if (ival >= 0 and ival < 10):
                        NodePIDMode.setProperty("mode").send(ival)

                elif (field == "Power"):
                    fval = float(value)
                    if (fval >= 0.0 and fval <= 1000.0):
                      NodeElectricityPower.setProperty("power").send(fval)

                elif (field == "Energy"):
                    fval = float(value)
                    if (fval >= 0.0 and fval <= 1E9):
                        # No real range validation possible; let OpenHab handle this
                        # as the previous value is known
                        NodeElectricityEnergy.setProperty("energy").send(float(value))

            except Exception as e:
                        print (e)




logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


config = homie.loadConfigFile("homie-python.json")
Homie = homie.Homie(config)


# temperature sensors
NodeSensor1Temperature = Homie.Node("sensor1", "temperature")
NodeSensor2Temperature = Homie.Node("sensor2", "temperature")
NodeSensor3Temperature = Homie.Node("sensor3", "temperature")

# pid controller
NodePIDMeasured = Homie.Node("pid-controller", "measured")
NodePIDSetpoint = Homie.Node("pid-controller", "setpoint")
NodePIDOutput   = Homie.Node("pid-controller", "output")
NodePIDMode     = Homie.Node("pid-controller", "mode")

# power/energy
NodeElectricityPower = Homie.Node("electricity", "power")
NodeElectricityEnergy = Homie.Node("electricity", "energy")


Homie.setFirmware("waterbed-controller", "1.0.0")

NodeSensor1Temperature.advertise("temperature")
NodeSensor1Temperature.advertise("unit")
NodeSensor1Temperature.advertise("datatype")

NodeSensor2Temperature.advertise("temperature")
NodeSensor2Temperature.advertise("unit")
NodeSensor2Temperature.advertise("datatype")

NodeSensor3Temperature.advertise("temperature")
NodeSensor3Temperature.advertise("unit")
NodeSensor3Temperature.advertise("datatype")


NodePIDMeasured.advertise("measured")
NodePIDMeasured.advertise("unit")
NodePIDMeasured.advertise("datatype")

NodePIDSetpoint.advertise("setpoint")
NodePIDSetpoint.advertise("unit")
NodePIDSetpoint.advertise("datatype")

NodePIDOutput.advertise("output")
NodePIDOutput.advertise("unit")
NodePIDOutput.advertise("datatype")

NodePIDMode.advertise("mode")
NodePIDMode.advertise("unit")
NodePIDMode.advertise("datatype")


NodeElectricityPower.advertise("power")
NodeElectricityPower.advertise("unit")
NodeElectricityPower.advertise("datatype")

NodeElectricityEnergy.advertise("energy").settable(EnergyUpdateInputHandler)
NodeElectricityEnergy.advertise("unit")
NodeElectricityEnergy.advertise("datatype")


#humidityNode.advertise("humidity")
print ("dbg: homie setup()...")
Homie.setup()
#time.sleep(2)   # give homie some time to connect (why not event based?)

NodeSensor1Temperature.setProperty("unit").send("°C")
NodeSensor1Temperature.setProperty("datatype").send("float")

NodeSensor2Temperature.setProperty("unit").send("°C")
NodeSensor2Temperature.setProperty("datatype").send("float")

NodeSensor3Temperature.setProperty("unit").send("°C")
NodeSensor3Temperature.setProperty("datatype").send("float")


NodePIDMeasured.setProperty("unit").send("°C")
NodePIDMeasured.setProperty("datatype").send("float")

NodePIDSetpoint.setProperty("unit").send("°C")
NodePIDSetpoint.setProperty("datatype").send("float")

NodePIDOutput.setProperty("unit").send("%")
NodePIDOutput.setProperty("datatype").send("float")

NodePIDMode.setProperty("unit").send("auto")
NodePIDMode.setProperty("datatype").send("boolean")


NodeElectricityPower.setProperty("unit").send("W")
NodeElectricityPower.setProperty("datatype").send("float")


NodeElectricityEnergy.setProperty("unit").send("Wh")
NodeElectricityEnergy.setProperty("datatype").send("float")




# FIXME: we should handle reconnects, network outage etc.
print ("opening socket...")
#esp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#esp_socket.connect(('192.168.6.61', 23))
esp_socket = open_esp_connection()
print("connected to ESP")

send_command("values")

if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, SystemExit):
        logger.info("Quitting.")
