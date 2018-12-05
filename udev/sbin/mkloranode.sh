#!/bin/sh

n=`cat /proc/devices | grep -c lora`

if [ "X$n" = "X0" ]
then
    echo "Modul nicht geladen!"
    exit 1
fi


major=`cat /proc/devices | grep lora | cut -d" " -f 1`

mknod /dev/rfm95w -m 666 c $major 0

exit 0
