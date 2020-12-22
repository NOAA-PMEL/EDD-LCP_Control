EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 13 20
Title "LCP Controller "
Date "2020-06-09"
Rev "0.1"
Comp "NOAA Pacific Marine Environmental Laboratory"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 "Current design by: Matt Casari"
$EndDescr
$Comp
L Mechanical:MountingHole H?
U 1 1 5EEE7614
P 4850 4550
AR Path="/60F8228B/5EEE7614" Ref="H?"  Part="1" 
AR Path="/5EEDF256/5EEE7614" Ref="H1"  Part="1" 
F 0 "H1" H 4950 4596 50  0000 L CNN
F 1 "IMU Mounting Hole #1" H 4950 4505 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm_Pad" H 4850 4550 50  0001 C CNN
F 3 "~" H 4850 4550 50  0001 C CNN
F 4 "N/A" H 4850 4550 50  0001 C CNN "MPN"
	1    4850 4550
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H?
U 1 1 5EEE761A
P 4850 4800
AR Path="/60F8228B/5EEE761A" Ref="H?"  Part="1" 
AR Path="/5EEDF256/5EEE761A" Ref="H2"  Part="1" 
F 0 "H2" H 4950 4846 50  0000 L CNN
F 1 "IMU Mounting Hole #2" H 4950 4755 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm_Pad" H 4850 4800 50  0001 C CNN
F 3 "~" H 4850 4800 50  0001 C CNN
F 4 "N/A" H 4850 4800 50  0001 C CNN "MPN"
	1    4850 4800
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H?
U 1 1 5EEE7620
P 5900 4550
AR Path="/60F8228B/5EEE7620" Ref="H?"  Part="1" 
AR Path="/5EEDF256/5EEE7620" Ref="H3"  Part="1" 
F 0 "H3" H 6000 4596 50  0000 L CNN
F 1 "IMU Mounting Hole #3" H 6000 4505 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm_Pad" H 5900 4550 50  0001 C CNN
F 3 "~" H 5900 4550 50  0001 C CNN
F 4 "N/A" H 5900 4550 50  0001 C CNN "MPN"
	1    5900 4550
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H?
U 1 1 5EEE7626
P 5900 4800
AR Path="/60F8228B/5EEE7626" Ref="H?"  Part="1" 
AR Path="/5EEDF256/5EEE7626" Ref="H4"  Part="1" 
F 0 "H4" H 6000 4846 50  0000 L CNN
F 1 "IMU Mounting Hole #4" H 6000 4755 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm_Pad" H 5900 4800 50  0001 C CNN
F 3 "~" H 5900 4800 50  0001 C CNN
F 4 "N/A" H 5900 4800 50  0001 C CNN "MPN"
	1    5900 4800
	1    0    0    -1  
$EndComp
Text Notes 6050 2400 2    200  ~ 0
IMU
$Comp
L Connector_Generic:Conn_01x04 J?
U 1 1 5EEE762D
P 7200 4200
AR Path="/60F8228B/5EEE762D" Ref="J?"  Part="1" 
AR Path="/5EEDF256/5EEE762D" Ref="J9"  Part="1" 
F 0 "J9" H 7118 3775 50  0000 C CNN
F 1 "Conn_01x04" H 7118 3866 50  0000 C CNN
F 2 "Connector_JST:JST_SH_BM04B-SRSS-TB_1x04-1MP_P1.00mm_Vertical" H 7200 4200 50  0001 C CNN
F 3 "~" H 7200 4200 50  0001 C CNN
F 4 "BM04B-SRSS-TB" H 7200 4200 50  0001 C CNN "MPN"
	1    7200 4200
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5EEE7633
P 6650 4400
AR Path="/60F8228B/5EEE7633" Ref="#PWR?"  Part="1" 
AR Path="/5EEDF256/5EEE7633" Ref="#PWR065"  Part="1" 
F 0 "#PWR065" H 6650 4150 50  0001 C CNN
F 1 "GND" H 6750 4300 50  0000 C CNN
F 2 "" H 6650 4400 50  0001 C CNN
F 3 "" H 6650 4400 50  0001 C CNN
	1    6650 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 4300 6650 4300
Wire Wire Line
	6650 4300 6650 4400
$Comp
L SparkFun-DiscreteSemi:MOSFET_PCH-DMG2307L Q?
U 1 1 5EEE763C
P 5450 3250
AR Path="/60F8228B/5EEE763C" Ref="Q?"  Part="1" 
AR Path="/5EEDF256/5EEE763C" Ref="Q8"  Part="1" 
F 0 "Q8" H 5615 3155 45  0000 L CNN
F 1 "DMG2305UX-7" H 5615 3239 45  0001 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5450 3500 20  0001 C CNN
F 3 "" H 5450 3250 50  0001 C CNN
F 4 "TRANS-11308" H 5615 3334 60  0001 L CNN "Field4"
F 5 "DMG2305UX-7" H 5450 3250 50  0001 C CNN "MPN"
	1    5450 3250
	1    0    0    1   
$EndComp
Wire Wire Line
	5550 3500 5550 3450
Wire Wire Line
	5800 4000 5800 3950
Wire Wire Line
	5700 4100 5700 3950
$Comp
L Device:R R?
U 1 1 5EEE764D
P 5300 2950
AR Path="/60F8228B/5EEE764D" Ref="R?"  Part="1" 
AR Path="/5EEDF256/5EEE764D" Ref="R24"  Part="1" 
F 0 "R24" V 5093 2950 50  0000 C CNN
F 1 "10k" V 5184 2950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5230 2950 50  0001 C CNN
F 3 "~" H 5300 2950 50  0001 C CNN
F 4 "	EXB-V4V103JV" H 5300 2950 50  0001 C CNN "MPN"
	1    5300 2950
	0    1    1    0   
$EndComp
Wire Wire Line
	5450 2950 5550 2950
Connection ~ 5550 2950
Wire Wire Line
	5550 2950 5550 3050
Wire Wire Line
	5250 3150 5100 3150
Wire Wire Line
	5100 3150 5100 2950
Wire Wire Line
	5100 2950 5150 2950
Connection ~ 5100 3150
Wire Wire Line
	6100 3500 6100 3900
Wire Wire Line
	5550 3500 5700 3500
Wire Wire Line
	5700 3550 5700 3500
Connection ~ 5700 3500
Wire Wire Line
	5700 3500 5800 3500
Wire Wire Line
	5800 3550 5800 3500
Connection ~ 5800 3500
Wire Wire Line
	5800 3500 6100 3500
$Comp
L Device:R_Pack02 RN?
U 1 1 5EEE7664
P 5700 3750
AR Path="/5ED049DE/5EEE7664" Ref="RN?"  Part="1" 
AR Path="/60F8228B/5EEE7664" Ref="RN?"  Part="1" 
AR Path="/5EEDF256/5EEE7664" Ref="RN17"  Part="1" 
F 0 "RN17" V 5383 3750 50  0000 C CNN
F 1 "10k" V 5474 3750 50  0000 C CNN
F 2 "Resistor_SMD:R_Array_Concave_2x0603" V 5875 3750 50  0001 C CNN
F 3 "~" H 5700 3750 50  0001 C CNN
F 4 "EXB-V4V103JV" H 5700 3750 50  0001 C CNN "MPN"
	1    5700 3750
	-1   0    0    1   
$EndComp
Text Notes 4600 5250 0    50   ~ 0
ToDo: Validate the Hole Size and Spacing for the Sparkfun Razor IMU\n1) https://www.sparkfun.com/products/15335     1"x1" board with 0.8" by 0.8" mounting holes\n2) https://www.sparkfun.com/products/16832  1.2"x1.2" board with 1"x1" mounting holes\n
$Comp
L power:GND #PWR?
U 1 1 5EEE766B
P 5900 3100
AR Path="/60F8228B/5EEE766B" Ref="#PWR?"  Part="1" 
AR Path="/5EEDF256/5EEE766B" Ref="#PWR064"  Part="1" 
F 0 "#PWR064" H 5900 2850 50  0001 C CNN
F 1 "GND" H 5800 3000 50  0000 C CNN
F 2 "" H 5900 3100 50  0001 C CNN
F 3 "" H 5900 3100 50  0001 C CNN
	1    5900 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 2750 6000 2750
Wire Wire Line
	6000 2850 5900 2850
Connection ~ 5900 2850
Wire Wire Line
	5900 2850 5900 2750
Wire Wire Line
	6000 2950 5900 2950
Wire Wire Line
	5900 2850 5900 2950
Connection ~ 5900 2950
Wire Wire Line
	5900 2950 5900 3050
Wire Wire Line
	6000 3050 5900 3050
Connection ~ 5900 3050
Wire Wire Line
	5900 3050 5900 3100
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J?
U 1 1 5EEE767C
P 6300 2850
AR Path="/5ED049DE/5EEE767C" Ref="J?"  Part="1" 
AR Path="/60F8228B/5EEE767C" Ref="J?"  Part="1" 
AR Path="/5EEDF256/5EEE767C" Ref="J8"  Part="1" 
F 0 "J8" H 6400 3050 50  0000 R CNN
F 1 "SALEAE_TEST" H 6550 2550 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 6300 2850 50  0001 C CNN
F 3 "~" H 6300 2850 50  0001 C CNN
F 4 "0010897080" H 6300 2850 50  0001 C CNN "MPN"
	1    6300 2850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6500 2750 6550 2750
Wire Wire Line
	6500 2950 6750 2950
Wire Wire Line
	6550 2750 6550 4000
Connection ~ 6550 4000
Wire Wire Line
	6550 4000 5800 4000
Wire Wire Line
	6500 2850 6650 2850
Wire Wire Line
	6650 2850 6650 4100
Connection ~ 6650 4100
Wire Wire Line
	6650 4100 5700 4100
Wire Wire Line
	6750 2950 6750 3900
Wire Wire Line
	6100 3900 6750 3900
Connection ~ 6750 3900
Wire Wire Line
	6750 3900 6750 4200
NoConn ~ 6500 3050
Text HLabel 3000 3150 0    50   Input ~ 0
IMU_ON
Text HLabel 3000 4000 0    50   Input ~ 0
IMU_SCL
Text HLabel 3000 4100 0    50   BiDi ~ 0
IMU_SDA
Wire Wire Line
	3000 4100 4300 4100
Connection ~ 5700 4100
Wire Wire Line
	5800 4000 4200 4000
Connection ~ 5800 4000
Wire Wire Line
	3000 3150 3750 3150
$Comp
L power:+3.3V #PWR?
U 1 1 5EEE7646
P 5550 2750
AR Path="/60F8228B/5EEE7646" Ref="#PWR?"  Part="1" 
AR Path="/5EEDF256/5EEE7646" Ref="#PWR063"  Part="1" 
F 0 "#PWR063" H 5550 2600 50  0001 C CNN
F 1 "+3.3V" H 5565 2923 50  0000 C CNN
F 2 "" H 5550 2750 50  0001 C CNN
F 3 "" H 5550 2750 50  0001 C CNN
	1    5550 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2750 5550 2950
$Comp
L Connector:TestPoint TP46
U 1 1 5EE382D0
P 6100 3500
F 0 "TP46" H 6158 3663 50  0000 L CNN
F 1 "5004" H 6158 3572 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 6300 3500 50  0001 C CNN
F 3 "~" H 6300 3500 50  0001 C CNN
F 4 "IMU_PWR" H 6158 3481 50  0000 L CNN "TestPoint"
F 5 "5004" H 6100 3500 50  0001 C CNN "MPN"
	1    6100 3500
	1    0    0    -1  
$EndComp
Connection ~ 6100 3500
$Comp
L Connector:TestPoint TP47
U 1 1 5EE3A0BE
P 8150 3500
F 0 "TP47" H 8208 3618 50  0000 L CNN
F 1 "5004" H 8208 3527 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 8350 3500 50  0001 C CNN
F 3 "~" H 8350 3500 50  0001 C CNN
F 4 "IMU_PWR" H 8150 3500 50  0001 C CNN "TestPoint"
F 5 "5004" H 8150 3500 50  0001 C CNN "MPN"
	1    8150 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5EE3A3D5
P 8150 3650
AR Path="/60F8228B/5EE3A3D5" Ref="#PWR?"  Part="1" 
AR Path="/5EEDF256/5EE3A3D5" Ref="#PWR0121"  Part="1" 
F 0 "#PWR0121" H 8150 3400 50  0001 C CNN
F 1 "GND" H 8250 3550 50  0000 C CNN
F 2 "" H 8150 3650 50  0001 C CNN
F 3 "" H 8150 3650 50  0001 C CNN
	1    8150 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 3650 8150 3500
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J?
U 1 1 5EF696EB
P 3700 4550
AR Path="/5ED049DE/5EF696EB" Ref="J?"  Part="1" 
AR Path="/60F8228B/5EF696EB" Ref="J?"  Part="1" 
AR Path="/5EDB548E/5EF696EB" Ref="J?"  Part="1" 
AR Path="/5EEDF256/5EF696EB" Ref="J32"  Part="1" 
F 0 "J32" H 3800 4750 50  0000 R CNN
F 1 "SALEAE_TEST" H 3950 4250 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 3700 4550 50  0001 C CNN
F 3 "~" H 3700 4550 50  0001 C CNN
F 4 "IMU" H 3700 4150 50  0000 C CNN "TestPoint"
F 5 "0010897080" H 3700 4550 50  0001 C CNN "MPN"
	1    3700 4550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3100 4750 3100 4800
Connection ~ 3100 4750
Wire Wire Line
	3400 4750 3100 4750
Wire Wire Line
	3100 4650 3100 4750
Connection ~ 3100 4650
Wire Wire Line
	3400 4650 3100 4650
Wire Wire Line
	3100 4550 3100 4650
Wire Wire Line
	3100 4550 3100 4450
Connection ~ 3100 4550
Wire Wire Line
	3400 4550 3100 4550
Wire Wire Line
	3100 4450 3400 4450
$Comp
L power:GND #PWR?
U 1 1 5EF696FC
P 3100 4800
AR Path="/60F8228B/5EF696FC" Ref="#PWR?"  Part="1" 
AR Path="/5EDB548E/5EF696FC" Ref="#PWR?"  Part="1" 
AR Path="/5EEDF256/5EF696FC" Ref="#PWR0120"  Part="1" 
F 0 "#PWR0120" H 3100 4550 50  0001 C CNN
F 1 "GND" H 3000 4700 50  0000 C CNN
F 2 "" H 3100 4800 50  0001 C CNN
F 3 "" H 3100 4800 50  0001 C CNN
	1    3100 4800
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP45
U 1 1 5EF6A26C
P 3750 3050
F 0 "TP45" H 3808 3213 50  0000 L CNN
F 1 "5002" H 3808 3122 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 3950 3050 50  0001 C CNN
F 3 "~" H 3950 3050 50  0001 C CNN
F 4 "IMU_ON" H 3808 3031 50  0000 L CNN "TestPoint"
F 5 "5002" H 3750 3050 50  0001 C CNN "MPN"
	1    3750 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 3050 3750 3150
Connection ~ 3750 3150
Wire Wire Line
	3750 3150 5100 3150
Wire Wire Line
	3900 4450 4100 4450
Wire Wire Line
	4100 4450 4100 3500
Wire Wire Line
	4100 3500 5550 3500
Connection ~ 5550 3500
Wire Wire Line
	3900 4550 4200 4550
Wire Wire Line
	4200 4550 4200 4000
Connection ~ 4200 4000
Wire Wire Line
	4200 4000 3000 4000
Wire Wire Line
	3900 4650 4300 4650
Wire Wire Line
	4300 4650 4300 4100
Connection ~ 4300 4100
Wire Wire Line
	4300 4100 5700 4100
NoConn ~ 3900 4750
Wire Wire Line
	6750 4200 7000 4200
Wire Wire Line
	6650 4100 7000 4100
Wire Wire Line
	6550 4000 7000 4000
Text Notes 7400 4350 0    50   ~ 0
Qwiic Cable:\n1- Black - GND\n2- Red - 3.3V\n3- Blue - SDA\n4- Yellow - SCL
Text Notes 5600 4000 1    50   ~ 0
RN17 DNP
$EndSCHEMATC
