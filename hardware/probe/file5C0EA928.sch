EESchema Schematic File Version 4
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 17
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L SparkFun-Resistors:RESISTOR1206 R?
U 1 1 5C0EAFBF
P 5600 3400
F 0 "R?" V 5500 3500 45  0000 L CNN
F 1 "1M" V 5600 3500 45  0000 L CNN
F 2 "1206" H 5600 3550 20  0001 C CNN
F 3 "" H 5600 3400 60  0001 C CNN
F 4 " " V 5684 3468 60  0000 L CNN "Field4"
	1    5600 3400
	0    1    1    0   
$EndComp
Text HLabel 5600 3200 0    50   Input ~ 0
INPUT
Text HLabel 5600 3600 0    50   Input ~ 0
OUTPUT
$Comp
L power:GND #PWR?
U 1 1 5C0EB6C7
P 5600 4000
F 0 "#PWR?" H 5600 4000 30  0001 C CNN
F 1 "GND" H 5600 3930 30  0001 C CNN
F 2 "" H 5600 4000 50  0001 C CNN
F 3 "" H 5600 4000 50  0001 C CNN
	1    5600 4000
	1    0    0    -1  
$EndComp
$Comp
L SparkFun-Resistors:10OHM-0603-1_10W-1% R?
U 1 1 5C0EB909
P 5600 3800
AR Path="/5C0EB909" Ref="R?"  Part="1" 
AR Path="/5C0EA929/5C0EB909" Ref="R?"  Part="1" 
F 0 "R?" V 5550 3900 45  0000 L CNN
F 1 "10k" V 5650 3900 45  0000 L CNN
F 2 "0603" H 5600 3950 20  0001 C CNN
F 3 "" H 5600 3800 60  0001 C CNN
F 4 "" V 5684 3868 60  0000 L CNN "Field4"
	1    5600 3800
	0    1    1    0   
$EndComp
$EndSCHEMATC
