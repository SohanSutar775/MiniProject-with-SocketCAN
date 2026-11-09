#!/usr/bin/env python3
"""Standalone ECU process.  Usage: python3 controller_ecu.py [can_interface]   (default vcan0)"""
from ecus import ControllerECU
from socketcan_io import run_ecu

if __name__ == "__main__":
    run_ecu(ControllerECU, hmi=False)
