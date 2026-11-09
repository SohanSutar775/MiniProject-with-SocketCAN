#!/usr/bin/env python3
"""Standalone ECU process.  Usage: python3 diagnostic_ecu.py [can_interface]   (default vcan0)"""
from ecus import DiagnosticECU
from socketcan_io import run_ecu

if __name__ == "__main__":
    run_ecu(DiagnosticECU, hmi=True)
