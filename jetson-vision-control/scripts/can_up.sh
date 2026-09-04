#!/bin/bash
# 125 kbps thay vi 500 kbps: module CAN (SN65HVD230) co R3=10k tren chan RS
# (slope-control mode, giu nguyen de chong nhieu EMI tu 3 servo) gioi han
# toc do bus an toan toi da. Chi tiet dieu tra + so lieu: validation/03-can/README.md
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 125000 sample-point 0.875 sjw 16 restart-ms 100
sudo ip link set can0 up
echo "can0 status:"
ip -details link show can0