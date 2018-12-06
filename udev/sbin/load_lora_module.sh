#!/bin/bash

n=`lsmod | grep -c lora_module`
[[ "X$n" = "X0" ]] || rmmod lora_module
modprobe lora_module
[[ -e /dev/rfm95w ]] && chmod 666 /dev/rfm95w

