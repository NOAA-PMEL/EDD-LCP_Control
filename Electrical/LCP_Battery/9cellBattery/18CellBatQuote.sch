EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L Device:Battery_Cell BAT1
U 1 1 5EBC2C7A
P 1800 2000
F 0 "BAT1" V 1545 2050 50  0000 C CNN
F 1 "ZEUS AA" V 1636 2050 50  0000 C CNN
F 2 "" V 1800 2060 50  0001 C CNN
F 3 "~" V 1800 2060 50  0001 C CNN
	1    1800 2000
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell BAT2
U 1 1 5EBC5444
P 2300 2000
F 0 "BAT2" V 2045 2050 50  0000 C CNN
F 1 "ZEUS AA" V 2136 2050 50  0000 C CNN
F 2 "" V 2300 2060 50  0001 C CNN
F 3 "~" V 2300 2060 50  0001 C CNN
	1    2300 2000
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell BAT8
U 1 1 5EBC621B
P 5600 2000
F 0 "BAT8" V 5345 2050 50  0000 C CNN
F 1 "ZEUS AA" V 5436 2050 50  0000 C CNN
F 2 "" V 5600 2060 50  0001 C CNN
F 3 "~" V 5600 2060 50  0001 C CNN
	1    5600 2000
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell BAT9
U 1 1 5EBC6E0E
P 6100 2000
F 0 "BAT9" V 5845 2050 50  0000 C CNN
F 1 "ZEUS AA" V 5936 2050 50  0000 C CNN
F 2 "" V 6100 2060 50  0001 C CNN
F 3 "~" V 6100 2060 50  0001 C CNN
	1    6100 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	2000 2000 2200 2000
Wire Wire Line
	5800 2000 6000 2000
$Comp
L Device:D_Schottky D1
U 1 1 5EBCCD50
P 6850 2000
F 0 "D1" H 6850 1784 50  0000 C CNN
F 1 "SB340" H 6850 1875 50  0000 C CNN
F 2 "" H 6850 2000 50  0001 C CNN
F 3 "~" H 6850 2000 50  0001 C CNN
	1    6850 2000
	-1   0    0    1   
$EndComp
Wire Wire Line
	6300 2000 6700 2000
Wire Wire Line
	7450 2100 7700 2100
Wire Wire Line
	7000 2000 7700 2000
$Comp
L Device:Battery_Cell BAT3
U 1 1 5EC71758
P 2850 2000
F 0 "BAT3" V 2595 2050 50  0000 C CNN
F 1 "ZEUS AA" V 2686 2050 50  0000 C CNN
F 2 "" V 2850 2060 50  0001 C CNN
F 3 "~" V 2850 2060 50  0001 C CNN
	1    2850 2000
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell BAT4
U 1 1 5EC71AE3
P 3350 2000
F 0 "BAT4" V 3095 2050 50  0000 C CNN
F 1 "ZEUS AA" V 3186 2050 50  0000 C CNN
F 2 "" V 3350 2060 50  0001 C CNN
F 3 "~" V 3350 2060 50  0001 C CNN
	1    3350 2000
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell BAT5
U 1 1 5EC71F75
P 3950 2000
F 0 "BAT5" V 3695 2050 50  0000 C CNN
F 1 "ZEUS AA" V 3786 2050 50  0000 C CNN
F 2 "" V 3950 2060 50  0001 C CNN
F 3 "~" V 3950 2060 50  0001 C CNN
	1    3950 2000
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell BAT6
U 1 1 5EC72426
P 4450 2000
F 0 "BAT6" V 4195 2050 50  0000 C CNN
F 1 "ZEUS AA" V 4286 2050 50  0000 C CNN
F 2 "" V 4450 2060 50  0001 C CNN
F 3 "~" V 4450 2060 50  0001 C CNN
	1    4450 2000
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell BAT7
U 1 1 5EC72953
P 5000 2000
F 0 "BAT7" V 4745 2050 50  0000 C CNN
F 1 "ZEUS AA" V 4836 2050 50  0000 C CNN
F 2 "" V 5000 2060 50  0001 C CNN
F 3 "~" V 5000 2060 50  0001 C CNN
	1    5000 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	2500 2000 2750 2000
Wire Wire Line
	3050 2000 3250 2000
Wire Wire Line
	3550 2000 3850 2000
Wire Wire Line
	4150 2000 4350 2000
Wire Wire Line
	4650 2000 4900 2000
Wire Wire Line
	5200 2000 5500 2000
Wire Wire Line
	7450 2200 7450 2100
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 5EC7424F
P 7900 2100
F 0 "J1" H 8008 2281 50  0000 C CNN
F 1 "Conn_01x02_Male" H 8008 2190 50  0000 C CNN
F 2 "" H 7900 2100 50  0001 C CNN
F 3 "~" H 7900 2100 50  0001 C CNN
	1    7900 2100
	-1   0    0    1   
$EndComp
Wire Wire Line
	1600 2000 1600 2200
Wire Wire Line
	1600 2000 1700 2000
Wire Wire Line
	1600 2200 7450 2200
$EndSCHEMATC