#!/bin/bash
# Start the four ECUs as independent processes on vcan0. Diagnostic ECU (HMI) runs in the foreground.
IF=${1:-vcan0}
python3 sensor_ecu.py $IF & P1=$!
python3 controller_ecu.py $IF & P2=$!
python3 actuator_ecu.py $IF & P3=$!
trap "kill $P1 $P2 $P3 2>/dev/null" EXIT
python3 diagnostic_ecu.py $IF
