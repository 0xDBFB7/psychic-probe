#!/bin/bash
echo "################WARNING###########"
echo "################WARNING###########"
echo "################WARNING###########"
read -p "DID YOU INCREMENT SERIAL NUMBER?"
make clean
make all -j16
make upload
