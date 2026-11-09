#!/usr/bin/env python3
"""Standalone ECU process.  Usage: python3 actuator_ecu.py [can_interface]   (default vcan0)"""
from ecus import ActuatorECU
from socketcan_io import run_ecu

if __name__ == "__main__":
    run_ecu(ActuatorECU, hmi=False)
