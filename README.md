# Vehicle Monitoring System (VMS) - Distributed system on SocketCAN

Four independent ECU processes (Sensor, Controller, Actuator, Diagnostic+HMI) exchange 10 CAN messages on `vcan0`.
Python 3.8+ standard library only (raw `AF_CAN` sockets) - no pip packages needed.

## Project structure
| File | Purpose |
|---|---|
| can_matrix.py | CAN IDs, CAN matrix, encode/decode functions, plausibility limits |
| ecus.py | ECU logic classes (hardware independent) |
| socketcan_io.py | SocketCAN backend + process runner (+ console HMI loop) |
| sensor_ecu.py / controller_ecu.py / actuator_ecu.py / diagnostic_ecu.py | Entry points - one OS process per ECU |
| sim_bus.py, analyse.py, figs.py | Virtual-time bus simulation, observation extraction, figures (no CAN hardware needed) |
| test_vms.py | Unit + integration tests |
| setup_vcan.sh, run_all.sh | Create vcan0, start the system |

## Build / run (Linux)
```
sudo ./setup_vcan.sh vcan0          # modprobe vcan; ip link add vcan0 type vcan; ip link set up vcan0
./run_all.sh vcan0                  # starts 3 ECUs in background + Diagnostic ECU / HMI in foreground
candump -td vcan0                   # (another terminal) live CAN traffic with delta time
```
Run each ECU manually in its own terminal instead: `python3 sensor_ecu.py vcan0`, etc.

## Fault injection (type key + Enter in the Diagnostic ECU window; sent as CAN frame 0x7E1)
`1` coolant = 200 C | `2` battery = 100 V | `3` mute Sensor ECU | `4` coolant = 108 C | `5` coolant = 118 C | `0` clear all.
Node failure: `Ctrl+C` (or `kill`) the Actuator or Controller process; recovery: start it again.

## Tests (no hardware)
```
python3 -m unittest -v test_vms     # 11 tests
python3 analyse.py                  # regenerates results.json (all observations)
```
