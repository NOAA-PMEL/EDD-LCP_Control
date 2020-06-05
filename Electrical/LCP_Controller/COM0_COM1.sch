EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 8 20
Title "LCP Controller "
Date "2020-06-04"
Rev "0.1"
Comp "NOAA Pacific Marine Environmental Laboratory"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 2600 2450 0    50   Input ~ 0
COM0_UART_TX
Text HLabel 2600 2550 0    50   Output ~ 0
COM0_UART_RX
Text HLabel 2600 2900 0    50   Input ~ 0
COM1_UART_TX
Text HLabel 2600 3000 0    50   Output ~ 0
COM1_UART_RX
Text HLabel 2600 1900 0    50   Input ~ 0
COM01_ON
Text HLabel 2600 2000 0    50   Input ~ 0
COM01_OFF
Text HLabel 2600 2100 0    50   Input ~ 0
~COM01_INV
$Comp
L Device:C_Small C?
U 1 1 5EEA7D58
P 6400 1800
AR Path="/5ED049DE/5EEA7D58" Ref="C?"  Part="1" 
AR Path="/5EDB3B75/5EEA7D58" Ref="C16"  Part="1" 
F 0 "C16" H 6492 1846 50  0000 L CNN
F 1 "0.1uF" H 6492 1755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6400 1800 50  0001 C CNN
F 3 "~" H 6400 1800 50  0001 C CNN
	1    6400 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 1650 6400 1650
Wire Wire Line
	6400 1450 6650 1450
Wire Wire Line
	6650 1450 6650 1500
Wire Wire Line
	6650 1500 6850 1500
Wire Wire Line
	6400 1900 6850 1900
Wire Wire Line
	6400 1700 6650 1700
Wire Wire Line
	6650 1700 6650 1750
Wire Wire Line
	6650 1750 6850 1750
Wire Wire Line
	6400 1950 6650 1950
Wire Wire Line
	6650 1950 6650 2000
Wire Wire Line
	6650 2000 6850 2000
$Comp
L Device:C_Small C?
U 1 1 5EEA7D69
P 6900 1000
AR Path="/5ED049DE/5EEA7D69" Ref="C?"  Part="1" 
AR Path="/5EDB3B75/5EEA7D69" Ref="C18"  Part="1" 
F 0 "C18" H 6992 1046 50  0000 L CNN
F 1 "0.1uF" H 6992 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6900 1000 50  0001 C CNN
F 3 "~" H 6900 1000 50  0001 C CNN
	1    6900 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5EEA7D6F
P 8550 1650
AR Path="/5ED049DE/5EEA7D6F" Ref="C?"  Part="1" 
AR Path="/5EDB3B75/5EEA7D6F" Ref="C20"  Part="1" 
F 0 "C20" H 8642 1696 50  0000 L CNN
F 1 "0.1uF" H 8642 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8550 1650 50  0001 C CNN
F 3 "~" H 8550 1650 50  0001 C CNN
	1    8550 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5EEA7D75
P 8100 1950
AR Path="/5ED049DE/5EEA7D75" Ref="C?"  Part="1" 
AR Path="/5EDB3B75/5EEA7D75" Ref="C19"  Part="1" 
F 0 "C19" H 8192 1996 50  0000 L CNN
F 1 "0.1uF" H 8192 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8100 1950 50  0001 C CNN
F 3 "~" H 8100 1950 50  0001 C CNN
	1    8100 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 1850 8100 1850
Wire Wire Line
	7300 1300 7300 900 
Wire Wire Line
	7300 900  7600 900 
Wire Wire Line
	7600 900  7600 1300
Connection ~ 7300 900 
$Comp
L power:GND #PWR?
U 1 1 5EEA7D81
P 6900 1150
AR Path="/5ED049DE/5EEA7D81" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B75/5EEA7D81" Ref="#PWR031"  Part="1" 
F 0 "#PWR031" H 6900 900 50  0001 C CNN
F 1 "GND" H 6905 977 50  0000 C CNN
F 2 "" H 6900 1150 50  0001 C CNN
F 3 "" H 6900 1150 50  0001 C CNN
	1    6900 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5EEA7D87
P 7450 3500
AR Path="/5ED049DE/5EEA7D87" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B75/5EEA7D87" Ref="#PWR032"  Part="1" 
F 0 "#PWR032" H 7450 3250 50  0001 C CNN
F 1 "GND" H 7455 3327 50  0000 C CNN
F 2 "" H 7450 3500 50  0001 C CNN
F 3 "" H 7450 3500 50  0001 C CNN
	1    7450 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5EEA7D8D
P 8100 2100
AR Path="/5ED049DE/5EEA7D8D" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B75/5EEA7D8D" Ref="#PWR033"  Part="1" 
F 0 "#PWR033" H 8100 1850 50  0001 C CNN
F 1 "GND" H 8105 1927 50  0000 C CNN
F 2 "" H 8100 2100 50  0001 C CNN
F 3 "" H 8100 2100 50  0001 C CNN
	1    8100 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5EEA7D93
P 8550 1800
AR Path="/5ED049DE/5EEA7D93" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B75/5EEA7D93" Ref="#PWR034"  Part="1" 
F 0 "#PWR034" H 8550 1550 50  0001 C CNN
F 1 "GND" H 8555 1627 50  0000 C CNN
F 2 "" H 8550 1800 50  0001 C CNN
F 3 "" H 8550 1800 50  0001 C CNN
	1    8550 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 1750 8550 1800
Wire Wire Line
	8100 2050 8100 2100
Wire Wire Line
	6900 1150 6900 1100
$Comp
L Device:C_Small C?
U 1 1 5EEA7D9D
P 6400 2050
AR Path="/5ED049DE/5EEA7D9D" Ref="C?"  Part="1" 
AR Path="/5EDB3B75/5EEA7D9D" Ref="C17"  Part="1" 
F 0 "C17" H 6492 2096 50  0000 L CNN
F 1 "0.1uF" H 6492 2005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6400 2050 50  0001 C CNN
F 3 "~" H 6400 2050 50  0001 C CNN
	1    6400 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 2150 6850 2150
Wire Wire Line
	8350 3000 8000 3000
Wire Wire Line
	8000 2900 8450 2900
Wire Wire Line
	8000 2800 8550 2800
Wire Wire Line
	8000 1550 8550 1550
$Comp
L Device:C_Small C?
U 1 1 5EEA7DAA
P 6400 1550
AR Path="/5ED049DE/5EEA7DAA" Ref="C?"  Part="1" 
AR Path="/5EDB3B75/5EEA7DAA" Ref="C15"  Part="1" 
F 0 "C15" H 6492 1596 50  0000 L CNN
F 1 "0.1uF" H 6492 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6400 1550 50  0001 C CNN
F 3 "~" H 6400 1550 50  0001 C CNN
	1    6400 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 2700 8650 2700
$Comp
L Device:R_Pack02 RN?
U 1 1 5EEA7DB2
P 5000 2550
AR Path="/5ED049DE/5EEA7DB2" Ref="RN?"  Part="1" 
AR Path="/5EDB3B75/5EEA7DB2" Ref="RN5"  Part="1" 
F 0 "RN5" V 4683 2550 50  0000 C CNN
F 1 "Jumper/0Ohm(x2)" V 4774 2550 50  0000 C CNN
F 2 "Resistor_SMD:R_Array_Concave_2x0603" V 5175 2550 50  0001 C CNN
F 3 "~" H 5000 2550 50  0001 C CNN
F 4 "YC122-JR-070RL" V 5000 2550 50  0001 C CNN "MPN"
	1    5000 2550
	0    1    1    0   
$EndComp
$Comp
L Device:R_Pack02 RN?
U 1 1 5EEA7DB8
P 5000 3000
AR Path="/5ED049DE/5EEA7DB8" Ref="RN?"  Part="1" 
AR Path="/5EDB3B75/5EEA7DB8" Ref="RN6"  Part="1" 
F 0 "RN6" V 4683 3000 50  0000 C CNN
F 1 "Jumper/0Ohm(x2)" V 4774 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_Array_Concave_2x0603" V 5175 3000 50  0001 C CNN
F 3 "~" H 5000 3000 50  0001 C CNN
	1    5000 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	5200 3000 6850 3000
Wire Wire Line
	5200 2900 6850 2900
Wire Wire Line
	5200 2550 5300 2550
Wire Wire Line
	5300 2550 5300 2800
Wire Wire Line
	5300 2800 6850 2800
Wire Wire Line
	5200 2450 5400 2450
Wire Wire Line
	5400 2450 5400 2700
Wire Wire Line
	5400 2700 6850 2700
Connection ~ 4150 3000
Wire Wire Line
	4150 3000 4800 3000
Connection ~ 4250 2900
Wire Wire Line
	4250 2900 4800 2900
Connection ~ 4350 2550
Wire Wire Line
	4350 2550 4800 2550
Connection ~ 4450 2450
Wire Wire Line
	4450 2450 4800 2450
$Comp
L Device:R_Pack02 RN?
U 1 1 5EEA7DE5
P 5000 4900
AR Path="/5ED049DE/5EEA7DE5" Ref="RN?"  Part="1" 
AR Path="/5EDB3B75/5EEA7DE5" Ref="RN8"  Part="1" 
F 0 "RN8" V 4683 4900 50  0000 C CNN
F 1 "Jumper/0Ohm(x2)" V 4774 4900 50  0000 C CNN
F 2 "Resistor_SMD:R_Array_Concave_2x0603" V 5175 4900 50  0001 C CNN
F 3 "~" H 5000 4900 50  0001 C CNN
	1    5000 4900
	0    1    1    0   
$EndComp
$Comp
L Device:R_Pack02 RN?
U 1 1 5EEA7DEB
P 5000 4450
AR Path="/5ED049DE/5EEA7DEB" Ref="RN?"  Part="1" 
AR Path="/5EDB3B75/5EEA7DEB" Ref="RN7"  Part="1" 
F 0 "RN7" V 4683 4450 50  0000 C CNN
F 1 "Jumper/0Ohm(x2)" V 4774 4450 50  0000 C CNN
F 2 "Resistor_SMD:R_Array_Concave_2x0603" V 5175 4450 50  0001 C CNN
F 3 "~" H 5000 4450 50  0001 C CNN
	1    5000 4450
	0    1    1    0   
$EndComp
Wire Wire Line
	5200 4900 8350 4900
Wire Wire Line
	5200 4800 8450 4800
Wire Wire Line
	5200 4450 5450 4450
Wire Wire Line
	5450 4450 5450 4700
Wire Wire Line
	5450 4700 8550 4700
Wire Wire Line
	5200 4350 5550 4350
Wire Wire Line
	5550 4350 5550 4600
Wire Wire Line
	5550 4600 8650 4600
Text Notes 4400 1650 0    67   ~ 0
For RS232 (COM0/1) Populate \nthese Resistor netorks 
$Comp
L PMEL_DriverICs:TRS3122E U?
U 1 1 5EEA7E05
P 7450 2550
AR Path="/5ED049DE/5EEA7E05" Ref="U?"  Part="1" 
AR Path="/5EDB3B75/5EEA7E05" Ref="U6"  Part="1" 
F 0 "U6" H 7800 3750 50  0000 C CNN
F 1 "TRS3122E" H 7750 1750 50  0000 C CNN
F 2 "Package_DFN_QFN:Texas_RGE0024H_EP2.7x2.7mm_ThermalVias" H 6700 3400 50  0001 C CNN
F 3 "https://www.ti.com/lit/ds/symlink/trs3122e.pdf?ts=1591021643258" H 6700 3400 50  0001 C CNN
	1    7450 2550
	1    0    0    -1  
$EndComp
Text Notes 4900 4100 0    100  ~ 20
COM0/1 UART
Text Notes 4550 1050 0    100  ~ 20
COM0/1 RS-232
Wire Wire Line
	2600 2450 4450 2450
Wire Wire Line
	2600 2550 4350 2550
Wire Wire Line
	2600 2900 4250 2900
Wire Wire Line
	2600 3000 4150 3000
Wire Wire Line
	2600 2100 5500 2100
Wire Wire Line
	5500 2100 5500 2500
Wire Wire Line
	5500 2500 6850 2500
Wire Wire Line
	5600 2400 5600 2000
Wire Wire Line
	5600 2000 2600 2000
Wire Wire Line
	5600 2400 6850 2400
Wire Wire Line
	2600 1900 5700 1900
Wire Wire Line
	5700 1900 5700 2300
Wire Wire Line
	5700 2300 6850 2300
Wire Wire Line
	4450 2450 4450 4350
Wire Wire Line
	4450 4350 4800 4350
Wire Wire Line
	4350 2550 4350 4450
Wire Wire Line
	4350 4450 4800 4450
Wire Wire Line
	4250 2900 4250 4800
Wire Wire Line
	4250 4800 4800 4800
Wire Wire Line
	4150 3000 4150 4900
Wire Wire Line
	4150 4900 4800 4900
Wire Wire Line
	8350 3000 8350 4900
Wire Wire Line
	8450 2900 8450 4800
Connection ~ 8450 4800
Wire Wire Line
	8550 2800 8550 4700
Connection ~ 8550 4700
Wire Wire Line
	8650 2700 8650 4600
Connection ~ 8650 4600
Text HLabel 10400 4600 2    50   Output ~ 0
COM0_TX
Text HLabel 10400 4700 2    50   Input ~ 0
COM0_RX
Text HLabel 10400 4800 2    50   Output ~ 0
COM1_TX
Text HLabel 10400 4900 2    50   Input ~ 0
COM1_RX
Wire Wire Line
	8550 4700 9200 4700
Wire Wire Line
	10400 4800 10000 4800
Wire Wire Line
	8350 4900 10100 4900
Connection ~ 8350 4900
Text HLabel 2650 5600 0    50   Input ~ 0
COM0_ON
Text HLabel 2650 6600 0    50   Input ~ 0
COM1_ON
Text HLabel 10400 5650 2    50   Output ~ 0
COM0_PWR
Text HLabel 10400 5850 2    50   Output ~ 0
COM1_PWR
$Comp
L SparkFun-DiscreteSemi:MOSFET_PCH-DMG2307L Q?
U 1 1 5F4567CC
P 3350 5700
AR Path="/60F8228B/5F4567CC" Ref="Q?"  Part="1" 
AR Path="/5EEDF256/5F4567CC" Ref="Q?"  Part="1" 
AR Path="/5EDB3B75/5F4567CC" Ref="Q4"  Part="1" 
F 0 "Q4" H 3515 5605 45  0000 L CNN
F 1 "MOSFET_PCH-DMG2305UX-7" H 3515 5689 45  0001 L CNN
F 2 "SOT23-3" H 3350 5950 20  0001 C CNN
F 3 "" H 3350 5700 50  0001 C CNN
F 4 "TRANS-11308" H 3515 5784 60  0001 L CNN "Field4"
	1    3350 5700
	1    0    0    1   
$EndComp
Wire Wire Line
	3450 5950 3450 5900
$Comp
L power:+3.3V #PWR?
U 1 1 5F4567D3
P 3450 5300
AR Path="/60F8228B/5F4567D3" Ref="#PWR?"  Part="1" 
AR Path="/5EEDF256/5F4567D3" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B75/5F4567D3" Ref="#PWR029"  Part="1" 
F 0 "#PWR029" H 3450 5150 50  0001 C CNN
F 1 "+3.3V" H 3465 5473 50  0000 C CNN
F 2 "" H 3450 5300 50  0001 C CNN
F 3 "" H 3450 5300 50  0001 C CNN
	1    3450 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 5300 3450 5400
$Comp
L Device:R R?
U 1 1 5F4567DA
P 3200 5400
AR Path="/60F8228B/5F4567DA" Ref="R?"  Part="1" 
AR Path="/5EEDF256/5F4567DA" Ref="R?"  Part="1" 
AR Path="/5EDB3B75/5F4567DA" Ref="R11"  Part="1" 
F 0 "R11" V 2993 5400 50  0000 C CNN
F 1 "10k" V 3084 5400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3130 5400 50  0001 C CNN
F 3 "~" H 3200 5400 50  0001 C CNN
	1    3200 5400
	0    1    1    0   
$EndComp
Wire Wire Line
	3350 5400 3450 5400
Connection ~ 3450 5400
Wire Wire Line
	3450 5400 3450 5500
Wire Wire Line
	3150 5600 3000 5600
Wire Wire Line
	3000 5600 3000 5400
Wire Wire Line
	3000 5400 3050 5400
Wire Wire Line
	2650 5600 3000 5600
Connection ~ 3000 5600
$Comp
L SparkFun-DiscreteSemi:MOSFET_PCH-DMG2307L Q?
U 1 1 5F45C958
P 3350 6700
AR Path="/60F8228B/5F45C958" Ref="Q?"  Part="1" 
AR Path="/5EEDF256/5F45C958" Ref="Q?"  Part="1" 
AR Path="/5EDB3B75/5F45C958" Ref="Q5"  Part="1" 
F 0 "Q5" H 3515 6605 45  0000 L CNN
F 1 "MOSFET_PCH-DMG2305UX-7" H 3515 6689 45  0001 L CNN
F 2 "SOT23-3" H 3350 6950 20  0001 C CNN
F 3 "" H 3350 6700 50  0001 C CNN
F 4 "TRANS-11308" H 3515 6784 60  0001 L CNN "Field4"
	1    3350 6700
	1    0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5F45C95F
P 3450 6300
AR Path="/60F8228B/5F45C95F" Ref="#PWR?"  Part="1" 
AR Path="/5EEDF256/5F45C95F" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B75/5F45C95F" Ref="#PWR030"  Part="1" 
F 0 "#PWR030" H 3450 6150 50  0001 C CNN
F 1 "+3.3V" H 3465 6473 50  0000 C CNN
F 2 "" H 3450 6300 50  0001 C CNN
F 3 "" H 3450 6300 50  0001 C CNN
	1    3450 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 6300 3450 6400
$Comp
L Device:R R?
U 1 1 5F45C966
P 3200 6400
AR Path="/60F8228B/5F45C966" Ref="R?"  Part="1" 
AR Path="/5EEDF256/5F45C966" Ref="R?"  Part="1" 
AR Path="/5EDB3B75/5F45C966" Ref="R12"  Part="1" 
F 0 "R12" V 2993 6400 50  0000 C CNN
F 1 "10k" V 3084 6400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3130 6400 50  0001 C CNN
F 3 "~" H 3200 6400 50  0001 C CNN
	1    3200 6400
	0    1    1    0   
$EndComp
Wire Wire Line
	3350 6400 3450 6400
Connection ~ 3450 6400
Wire Wire Line
	3450 6400 3450 6500
Wire Wire Line
	3150 6600 3000 6600
Wire Wire Line
	3000 6600 3000 6400
Wire Wire Line
	3000 6400 3050 6400
Wire Wire Line
	2650 6600 3000 6600
Connection ~ 3000 6600
Wire Wire Line
	3450 5950 4550 5950
Wire Wire Line
	4550 5950 4550 5650
Wire Wire Line
	3450 7000 4700 7000
Wire Wire Line
	4700 7000 4700 5850
Wire Wire Line
	4700 5850 6800 5850
Wire Wire Line
	3450 6900 3450 7000
Wire Wire Line
	10200 3300 10200 3200
Wire Wire Line
	10100 3200 10100 3300
Wire Wire Line
	10000 3300 10000 3200
Wire Wire Line
	9900 3200 9900 3300
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J?
U 1 1 5F84B732
P 10000 3600
AR Path="/5ED049DE/5F84B732" Ref="J?"  Part="1" 
AR Path="/5EE033BE/5F84B732" Ref="J?"  Part="1" 
AR Path="/5EDB3B75/5F84B732" Ref="J3"  Part="1" 
F 0 "J3" V 9900 3350 50  0000 R CNN
F 1 "Saleae Test" V 9800 3350 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 10000 3600 50  0001 C CNN
F 3 "~" H 10000 3600 50  0001 C CNN
F 4 "COM1 Test Points" V 9700 3150 50  0000 C CNN "Field4"
	1    10000 3600
	0    -1   -1   0   
$EndComp
NoConn ~ 10200 3800
Wire Wire Line
	9300 3300 9300 3200
Wire Wire Line
	9200 3200 9200 3300
Wire Wire Line
	9100 3300 9100 3200
Wire Wire Line
	9000 3200 9000 3300
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J?
U 1 1 5F86DBFA
P 9100 3600
AR Path="/5ED049DE/5F86DBFA" Ref="J?"  Part="1" 
AR Path="/5EE033BE/5F86DBFA" Ref="J?"  Part="1" 
AR Path="/5EDB3B75/5F86DBFA" Ref="J2"  Part="1" 
F 0 "J2" V 9000 3350 50  0000 R CNN
F 1 "Saleae Test" V 8900 3350 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 9100 3600 50  0001 C CNN
F 3 "~" H 9100 3600 50  0001 C CNN
F 4 "COM0 Test Points" V 8800 3150 50  0000 C CNN "Field4"
	1    9100 3600
	0    -1   -1   0   
$EndComp
NoConn ~ 9300 3800
Wire Wire Line
	4550 5650 6050 5650
Wire Wire Line
	9000 3800 9000 5650
Connection ~ 9000 5650
Wire Wire Line
	9000 5650 10400 5650
Wire Wire Line
	9100 3800 9100 4600
Wire Wire Line
	8650 4600 9100 4600
Connection ~ 9100 4600
Wire Wire Line
	9100 4600 10400 4600
Wire Wire Line
	9200 3800 9200 4700
Connection ~ 9200 4700
Wire Wire Line
	9200 4700 10400 4700
Wire Wire Line
	9900 3800 9900 5850
Connection ~ 9900 5850
Wire Wire Line
	9900 5850 10400 5850
Wire Wire Line
	10000 3800 10000 4800
Connection ~ 10000 4800
Wire Wire Line
	10000 4800 8450 4800
Wire Wire Line
	10100 3800 10100 4900
Connection ~ 10100 4900
Wire Wire Line
	10100 4900 10400 4900
$Comp
L power:GND #PWR?
U 1 1 5F88E19C
P 10550 3300
AR Path="/5ED049DE/5F88E19C" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B75/5F88E19C" Ref="#PWR035"  Part="1" 
F 0 "#PWR035" H 10550 3050 50  0001 C CNN
F 1 "GND" H 10555 3127 50  0000 C CNN
F 2 "" H 10550 3300 50  0001 C CNN
F 3 "" H 10550 3300 50  0001 C CNN
	1    10550 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10550 3300 10550 3200
Wire Wire Line
	10550 3200 10200 3200
Connection ~ 9100 3200
Wire Wire Line
	9100 3200 9000 3200
Connection ~ 9200 3200
Wire Wire Line
	9200 3200 9100 3200
Connection ~ 9300 3200
Wire Wire Line
	9300 3200 9200 3200
Connection ~ 9900 3200
Wire Wire Line
	9900 3200 9300 3200
Connection ~ 10000 3200
Wire Wire Line
	10000 3200 9900 3200
Connection ~ 10100 3200
Wire Wire Line
	10100 3200 10000 3200
Connection ~ 10200 3200
Wire Wire Line
	10200 3200 10100 3200
Text HLabel 2550 1200 0    50   Input ~ 0
COM01_POWER
Wire Wire Line
	2550 1200 6450 1200
Wire Wire Line
	6450 1200 6450 900 
Wire Wire Line
	6450 900  6900 900 
Connection ~ 6900 900 
Wire Wire Line
	6900 900  7300 900 
Wire Wire Line
	7450 3350 7450 3500
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 6010CE46
P 6050 5600
F 0 "#FLG0104" H 6050 5675 50  0001 C CNN
F 1 "PWR_FLAG" H 6050 5773 50  0000 C CNN
F 2 "" H 6050 5600 50  0001 C CNN
F 3 "~" H 6050 5600 50  0001 C CNN
	1    6050 5600
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0105
U 1 1 6010D12D
P 6800 5600
F 0 "#FLG0105" H 6800 5675 50  0001 C CNN
F 1 "PWR_FLAG" H 6800 5773 50  0000 C CNN
F 2 "" H 6800 5600 50  0001 C CNN
F 3 "~" H 6800 5600 50  0001 C CNN
	1    6800 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 5600 6800 5850
Connection ~ 6800 5850
Wire Wire Line
	6800 5850 9900 5850
Wire Wire Line
	6050 5600 6050 5650
Connection ~ 6050 5650
Wire Wire Line
	6050 5650 9000 5650
$EndSCHEMATC