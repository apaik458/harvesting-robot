#!/usr/bin/env bash
# main.cpp only accepts a GUI connection for ~1s after startup, so start
# it first and give it a moment before the GUI tries to connect.
./build/arm_control "$@" &
sleep 0.5
python3 gui/gui.py
