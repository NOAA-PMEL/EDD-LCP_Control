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
Text GLabel 6100 3350 2    50   Input ~ 0
GPS_BATT_BCKP
$Comp
L Device:Battery_Cell BT1
U 1 1 6015D405
P 5750 3700
F 0 "BT1" H 5868 3796 50  0000 L CNN
F 1 "Battery_Cell" H 5868 3705 50  0000 L CNN
F 2 "Batteries:BATTCOM_20MM_PTH" V 5750 3760 50  0001 C CNN
F 3 "~" V 5750 3760 50  0001 C CNN
	1    5750 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR099
U 1 1 6015D832
P 5750 4000
F 0 "#PWR099" H 5750 3750 50  0001 C CNN
F 1 "GND" H 5755 3827 50  0000 C CNN
F 2 "" H 5750 4000 50  0001 C CNN
F 3 "" H 5750 4000 50  0001 C CNN
	1    5750 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 4000 5750 3800
Wire Wire Line
	5750 3500 5750 3350
Wire Wire Line
	5750 3350 6100 3350
$Comp
L power:PWR_FLAG #FLG016
U 1 1 6015EC71
P 5750 3350
F 0 "#FLG016" H 5750 3425 50  0001 C CNN
F 1 "PWR_FLAG" H 5750 3523 50  0000 C CNN
F 2 "" H 5750 3350 50  0001 C CNN
F 3 "~" H 5750 3350 50  0001 C CNN
	1    5750 3350
	1    0    0    -1  
$EndComp
Connection ~ 5750 3350
$EndSCHEMATC
