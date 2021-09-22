EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 11 19
Title "LCP Controller "
Date "2020-06-09"
Rev "0.1"
Comp "NOAA Pacific Marine Environmental Laboratory"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 "Current design by: Matt Casari"
$EndDescr
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
F 4 "BM04B-SRSS-TBT(LF)(SN)" H 7200 4200 50  0001 C CNN "MPN"
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
F 1 "TSM500P02CX RFG" H 5615 3239 45  0001 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5450 3500 20  0001 C CNN
F 3 "" H 5450 3250 50  0001 C CNN
F 4 "TRANS-11308" H 5615 3334 60  0001 L CNN "Field4"
F 5 "TSM500P02CX RFG" H 5450 3250 50  0001 C CNN "MPN"
	1    5450 3250
	1    0    0    1   
$EndComp
Wire Wire Line
	5550 3500 5550 3450
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
F 4 "ERA-6ARW103V" H 5300 2950 50  0001 C CNN "MPN"
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
	6100 3900 6750 3900
Wire Wire Line
	6750 3900 6750 4200
Text HLabel 3000 3150 0    50   Input ~ 0
IMU_ON
Text HLabel 3000 4000 0    50   Input ~ 0
IMU_SCL
Text HLabel 3000 4100 0    50   BiDi ~ 0
IMU_SDA
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
L Connector:TestPoint IMU_PWR1
U 1 1 5EE382D0
P 6100 3500
F 0 "IMU_PWR1" H 6158 3663 50  0000 L CNN
F 1 "5004" H 6158 3572 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 6300 3500 50  0001 C CNN
F 3 "~" H 6300 3500 50  0001 C CNN
F 4 "IMU_PWR" H 6158 3481 50  0000 L CNN "TestPoint"
F 5 "" H 6100 3500 50  0001 C CNN "MPN"
	1    6100 3500
	1    0    0    -1  
$EndComp
Connection ~ 6100 3500
$Comp
L Connector:TestPoint IMU_ON1
U 1 1 5EF6A26C
P 3750 3050
F 0 "IMU_ON1" H 3808 3213 50  0000 L CNN
F 1 "5002" H 3808 3122 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 3950 3050 50  0001 C CNN
F 3 "~" H 3950 3050 50  0001 C CNN
F 4 "IMU_ON" H 3808 3031 50  0000 L CNN "TestPoint"
F 5 "" H 3750 3050 50  0001 C CNN "MPN"
	1    3750 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 3050 3750 3150
Connection ~ 3750 3150
Wire Wire Line
	3750 3150 5100 3150
Wire Wire Line
	6750 4200 7000 4200
Text Notes 7400 4350 0    50   ~ 0
Qwiic Cable:\n1- Black - GND\n2- Red - 3.3V\n3- Blue - SDA\n4- Yellow - SCL
Text Notes 4600 5250 0    50   ~ 0
ToDo: Validate the Hole Size and Spacing for the Sparkfun Razor IMU\n1) https://www.sparkfun.com/products/15335     1"x1" board with 0.8" by 0.8" mounting holes\n2) https://www.sparkfun.com/products/16832  1.2"x1.2" board with 1"x1" mounting holes\n
Wire Wire Line
	3000 4000 7000 4000
Wire Wire Line
	3000 4100 7000 4100
Wire Wire Line
	5550 3500 6100 3500
$EndSCHEMATC
