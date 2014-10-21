#!/bin/bash

source /opt/ros/hydro/setup.bash
rostopic pub --once /pi_cmd std_msgs/String adhocwireless

IPSUFFIX=`echo  $1 | cut -d'.' -f 4`
IPSUFFIX_ORIG=$IPSUFFIX
IPSUFFIX=`expr $IPSUFFIX - 100`
CHANNEL=`expr $IPSUFFIX - 99`

echo "Starting up adhoc wireless"
echo "Make sure to use sudo when executing"
echo ""

iwlist wlan1 scan
stop network-manager
ifconfig wlan1 down
sleep 2
iwconfig wlan1 mode ad-hoc
iwconfig wlan1 channel $CHANNEL
iwconfig wlan1 key aaaaa11111
iwconfig wlan1 essid RPi$IPSUFFIX_ORIG
sleep 2
ifconfig wlan1 192.168.17.$IPSUFFIX netmask 255.255.254.0 broadcast 192.168.17.255  up

ping $1
