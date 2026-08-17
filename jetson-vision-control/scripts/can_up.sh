#!/bin/bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
echo "can0 status:"
ip -details link show can0