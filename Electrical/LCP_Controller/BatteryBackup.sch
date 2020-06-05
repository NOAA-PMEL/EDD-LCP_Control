EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 20 20
Title "LCP Controller "
Date "2020-06-04"
Rev "0.1"
Comp "NOAA Pacific Marine Environmental Laboratory"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 4350 2850 2    50   Input ~ 0
GPS_BATT_BCKP
$Comp
L Device:Battery_Cell BT1
U 1 1 6015D405
P 4000 3200
F 0 "BT1" H 4118 3296 50  0000 L CNN
F 1 "Battery_Cell" H 4118 3205 50  0000 L CNN
F 2 "" V 4000 3260 50  0001 C CNN
F 3 "~" V 4000 3260 50  0001 C CNN
	1    4000 3200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 6015D832
P 4000 3500
F 0 "#PWR0101" H 4000 3250 50  0001 C CNN
F 1 "GND" H 4005 3327 50  0000 C CNN
F 2 "" H 4000 3500 50  0001 C CNN
F 3 "" H 4000 3500 50  0001 C CNN
	1    4000 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 3500 4000 3300
Wire Wire Line
	4000 3000 4000 2850
Wire Wire Line
	4000 2850 4350 2850
$Comp
L power:PWR_FLAG #FLG0109
U 1 1 6015EC71
P 4000 2850
F 0 "#FLG0109" H 4000 2925 50  0001 C CNN
F 1 "PWR_FLAG" H 4000 3023 50  0000 C CNN
F 2 "" H 4000 2850 50  0001 C CNN
F 3 "~" H 4000 2850 50  0001 C CNN
	1    4000 2850
	1    0    0    -1  
$EndComp
Connection ~ 4000 2850
$EndSCHEMATC
