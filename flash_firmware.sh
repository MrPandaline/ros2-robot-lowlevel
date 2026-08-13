#!/bin/bash
set -e

arm-none-eabi-objcopy -O binary Debug/ros2-robot.elf Debug/ros2-robot.bin
scp Debug/ros2-robot.bin orangepi@orangepi4pro.local:/tmp/
ssh orangepi@orangepi4pro.local "st-flash --reset write /tmp/ros2-robot.bin 0x8000000"
