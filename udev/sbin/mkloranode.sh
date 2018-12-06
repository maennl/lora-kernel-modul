#!/bin/sh

DEV=rfm95w

n=`cat /proc/devices | grep -c $DEV`

if [ "X$n" = "X0" ]
then
    echo "Modul nicht geladen!"
    exit 1
fi


major=`cat /proc/devices | grep $DEV | cut -d" " -f 1`

mknod /dev/rfm95w -m 666 c $major 0 || true

chmod 666 /dev/rfm95w

exit 0
