# NANOPI NEO3 DHT11 access via gpio

#
#__author__="nextEDGE Technology"
#__copyright__ = "Copyright 2025 nxtEDGE Technology"
#__license__ = "public domain"
#__version__ = "1.0"

## Introduction
This includes GPIO sample codes for NANOPI NEO3 configured with Ubuntu 20.04.
Sample code is degied to use DHT11 with NANOPI NEO3 via mmap i/o.
Also includes simple GPIO High/Low test sample.

## System requirements
[NANOPI NEO3] (https://wiki.friendlyelec.com/wiki/index.php/NanoPi_NEO3)

rk3328-sd-friendlycore-lite-focal-5.10-arm64-20210830.img

Distributor ID: Ubuntu

Description:    Ubuntu 20.04.6 LTS

Release:        20.04

Codename:       focal

## packages
Install the following packages.
```bash
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install gdb
sudo apt-get install rt-tests
```

## Contents
```bash
gpio.c
dht11_mmap.c

:build & run
$gcc -O2 -o dht11_mmap dht11_mmap.c
$sudo ./dht11_mmap
```

## How to connect DHT11
PIN 2 5V, PIN 6 GND, PIN 7 DATA

![](/assets/NANOPI.jpg)
![](/assets/GPIO.jpg)
