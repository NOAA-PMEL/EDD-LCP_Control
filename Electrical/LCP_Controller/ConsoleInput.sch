EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 11 20
Title "LCP Controller "
Date "2020-06-04"
Rev "0.1"
Comp "NOAA Pacific Marine Environmental Laboratory"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 4300 3000 0    200  ~ 0
Console/Bootloader
Wire Wire Line
	5100 3950 5250 3950
Wire Wire Line
	5250 3950 5250 4050
Wire Wire Line
	5250 4050 5500 4050
$Comp
L power:GND #PWR?
U 1 1 5EE203FB
P 6000 4550
AR Path="/5EC7245E/5EE203FB" Ref="#PWR?"  Part="1" 
AR Path="/5EE01D96/5EE203FB" Ref="#PWR058"  Part="1" 
F 0 "#PWR058" H 6000 4300 50  0001 C CNN
F 1 "GND" H 6005 4377 50  0000 C CNN
F 2 "" H 6000 4550 50  0001 C CNN
F 3 "" H 6000 4550 50  0001 C CNN
	1    6000 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 4550 6000 4450
Wire Wire Line
	5800 4050 6000 4050
Wire Wire Line
	6000 4050 6000 4150
Connection ~ 6000 4050
Wire Wire Line
	6650 3650 5100 3650
Wire Wire Line
	5100 3450 5550 3450
Wire Wire Line
	5550 3450 5550 3300
Wire Wire Line
	6150 3300 6150 3400
NoConn ~ 5100 3550
Text Notes 4300 3000 0    200  ~ 0
Console/Bootloader
$Comp
L power:GND #PWR059
U 1 1 5EF59BE9
P 6000 4550
F 0 "#PWR059" H 6000 4300 50  0001 C CNN
F 1 "GND" H 6005 4377 50  0000 C CNN
F 2 "" H 6000 4550 50  0001 C CNN
F 3 "" H 6000 4550 50  0001 C CNN
	1    6000 4550
	1    0    0    -1  
$EndComp
Text Notes 4300 3000 0    200  ~ 0
Console/Bootloader
$Comp
L power:GND #PWR?
U 1 1 5EF59BEB
P 6000 4550
AR Path="/5EC7245E/5EF59BEB" Ref="#PWR?"  Part="1" 
AR Path="/5EE01D96/5EF59BEB" Ref="#PWR061"  Part="1" 
F 0 "#PWR061" H 6000 4300 50  0001 C CNN
F 1 "GND" H 6005 4377 50  0000 C CNN
F 2 "" H 6000 4550 50  0001 C CNN
F 3 "" H 6000 4550 50  0001 C CNN
	1    6000 4550
	1    0    0    -1  
$EndComp
Text Notes 4300 3000 0    200  ~ 0
Console/Bootloader
$Comp
L Device:C C32
U 1 1 5EF59BE3
P 5650 4050
F 0 "C32" V 5450 4050 50  0000 C CNN
F 1 "0.1uF" V 5350 4050 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5688 3900 50  0001 C CNN
F 3 "~" H 5650 4050 50  0001 C CNN
	1    5650 4050
	0    1    -1   0   
$EndComp
$Comp
L Device:R R21
U 1 1 5EF59BE7
P 6000 4300
F 0 "R21" H 6070 4346 50  0000 L CNN
F 1 "220k" H 6070 4255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5930 4300 50  0001 C CNN
F 3 "~" H 6000 4300 50  0001 C CNN
	1    6000 4300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR060
U 1 1 5EF59BEA
P 6000 4550
F 0 "#PWR060" H 6000 4300 50  0001 C CNN
F 1 "GND" H 6005 4377 50  0000 C CNN
F 2 "" H 6000 4550 50  0001 C CNN
F 3 "" H 6000 4550 50  0001 C CNN
	1    6000 4550
	1    0    0    -1  
$EndComp
Text GLabel 6650 3650 2    50   Input ~ 0
VCC_USB
$Comp
L power:GND #PWR062
U 1 1 5EF59BEE
P 6150 3400
F 0 "#PWR062" H 6150 3150 50  0001 C CNN
F 1 "GND" H 6155 3227 50  0000 C CNN
F 2 "" H 6150 3400 50  0001 C CNN
F 3 "" H 6150 3400 50  0001 C CNN
	1    6150 3400
	1    0    0    -1  
$EndComp
Text HLabel 6650 4050 2    50   Output ~ 0
CONSOLE_BOOT
Text HLabel 6650 3850 2    50   Input ~ 0
CONSOLE_TX
Text HLabel 6650 3750 2    50   Output ~ 0
CONSOLE_RX
Wire Wire Line
	5100 3750 6650 3750
Wire Wire Line
	5100 3850 6650 3850
Wire Wire Line
	6000 4050 6650 4050
$Comp
L Connector_Generic:Conn_01x06 J?
U 1 1 5EF59BE1
P 4900 3650
AR Path="/5EC7245E/5EF59BE1" Ref="J?"  Part="1" 
AR Path="/5EE01D96/5EF59BE1" Ref="J7"  Part="1" 
F 0 "J7" H 4900 4000 50  0000 C CNN
F 1 "Conn_01x06" H 4900 3200 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical" H 4900 3650 50  0001 C CNN
F 3 "~" H 4900 3650 50  0001 C CNN
	1    4900 3650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5550 3300 6150 3300
$EndSCHEMATC