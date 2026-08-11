#!/bin/bash
set -e

.elf Debug/ros2-robot.bin
scp Debug/ros2-robot.bin orangepi@orangepi4pro.local:/tmp/
ssh orangepi@orangepi4pro.local "st-flash --reset write /tmp/ros2-robot.bin 0x8000000"
