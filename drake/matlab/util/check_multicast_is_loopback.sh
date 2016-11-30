#!/bin/bash

loopback_interface=`ifconfig | grep -m 1 -i loopback | cut -d : -f1 | cut -d ' ' -f1`;
#echo loopback interface is $loopback_interface

if [ `ifconfig $loopback_interface | grep -c -i multicast` -eq 0 ]; then
  echo "  ERROR: Loopback interface is not set to multicast.";
  echo "  Consider running: sudo ./setup_loopback_multicast.sh";
  exit 1;
fi

if [ `uname -s` == "Darwin" ]; then
  if [ `route get 224.0.0.0 -netmask 240.0.0.0 | grep -m 1 -i interface | cut -f2 -d : | tr -d ' '` != $loopback_interface ]; then
    echo "  ERROR: Route to multicast channel does not run through the loopback interface.";
    echo "  Consider running: sudo ./setup_loopback_multicast.sh";
    exit 1;
  fi
else
  if [ `ip route get 224.0.0.0 | grep -m 1 -i dev | sed 's/.*dev\s*//g' | cut -d ' ' -f1` != $loopback_interface ]; then 
    echo "  ERROR: Route to multicast channel does not run through the loopback interface.";
    echo "  Consider running: sudo ./setup_loopback_multicast.sh";
    exit 1;
  fi
fi

exit 0;

