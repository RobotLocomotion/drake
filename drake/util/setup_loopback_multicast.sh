#!/bin/bash
# Note that you must run this script as superuser (e.g. using sudo) 

loopback_interface=`ifconfig | grep -m 1 -i loopback | cut -d : -f1 | cut -d ' ' -f1`;
#echo loopback interface is $loopback_interface

if [ `uname -s` == "Darwin" ]; then
  # loopback appears to be multicast by default...
  route add -net 224.0.0.0 -netmask 240.0.0.0 -interface $loopback_interface 
else
  ifconfig $loopback_interface multicast;
  route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
fi

