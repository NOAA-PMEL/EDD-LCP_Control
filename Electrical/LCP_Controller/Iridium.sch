EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 22 20
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
L power:GND #PWR?
U 1 1 5EE2A477
P 6550 5000
AR Path="/5EC7245E/5EE2A477" Ref="#PWR?"  Part="1" 
AR Path="/5EDA42CC/5EE2A477" Ref="#PWR044"  Part="1" 
F 0 "#PWR044" H 6550 4750 50  0001 C CNN
F 1 "GND" H 6555 4827 50  0000 C CNN
F 2 "" H 6550 5000 50  0001 C CNN
F 3 "" H 6550 5000 50  0001 C CNN
	1    6550 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 5000 6550 4950
Wire Wire Line
	6350 4850 6350 4950
Wire Wire Line
	6350 4950 6450 4950
Connection ~ 6550 4950
Wire Wire Line
	6550 4950 6550 4850
Wire Wire Line
	6750 4850 6750 4950
Wire Wire Line
	6750 4950 6650 4950
Wire Wire Line
	6650 4850 6650 4950
Connection ~ 6650 4950
Wire Wire Line
	6650 4950 6550 4950
Wire Wire Line
	6450 4850 6450 4950
Connection ~ 6450 4950
Wire Wire Line
	6450 4950 6550 4950
Text GLabel 6100 3400 0    50   Input ~ 0
EXT_PWR
Wire Wire Line
	6100 3400 6450 3400
Wire Wire Line
	6550 3500 6550 3400
Wire Wire Line
	6550 3400 6450 3400
Connection ~ 6450 3400
Wire Wire Line
	6450 3400 6450 3500
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J?
U 1 1 5EE2A49B
P 5100 3200
AR Path="/5ED049DE/5EE2A49B" Ref="J?"  Part="1" 
AR Path="/60F8228B/5EE2A49B" Ref="J?"  Part="1" 
AR Path="/5EC7245E/5EE2A49B" Ref="J?"  Part="1" 
AR Path="/5EDA42CC/5EE2A49B" Ref="J16"  Part="1" 
F 0 "J16" V 5250 2900 50  0000 R CNN
F 1 "SALEAE_TEST" V 5150 2900 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 5100 3200 50  0001 C CNN
F 3 "~" H 5100 3200 50  0001 C CNN
F 4 "Iridium COM Test" V 5050 2600 50  0000 C CNN "Field4"
	1    5100 3200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5000 3400 5000 3750
Connection ~ 5000 3750
Connection ~ 5100 3850
Wire Wire Line
	5100 3400 5100 3850
Wire Wire Line
	5200 3400 5200 3950
Connection ~ 5200 3950
Wire Wire Line
	5300 3400 5300 4450
$Comp
L power:GND #PWR?
U 1 1 5EE2A4AD
P 4700 3050
AR Path="/5EC7245E/5EE2A4AD" Ref="#PWR?"  Part="1" 
AR Path="/5EDA42CC/5EE2A4AD" Ref="#PWR042"  Part="1" 
F 0 "#PWR042" H 4700 2800 50  0001 C CNN
F 1 "GND" H 4705 2877 50  0000 C CNN
F 2 "" H 4700 3050 50  0001 C CNN
F 3 "" H 4700 3050 50  0001 C CNN
	1    4700 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 3050 4700 2800
Wire Wire Line
	5300 2800 5300 2900
Wire Wire Line
	5200 2900 5200 2800
Connection ~ 5200 2800
Wire Wire Line
	5200 2800 5300 2800
Wire Wire Line
	5100 2900 5100 2800
Connection ~ 5100 2800
Wire Wire Line
	5100 2800 5200 2800
Wire Wire Line
	5000 2900 5000 2800
Wire Wire Line
	4700 2800 5000 2800
Connection ~ 5000 2800
Wire Wire Line
	5000 2800 5100 2800
NoConn ~ 7200 3900
NoConn ~ 7200 4000
NoConn ~ 7200 4100
NoConn ~ 7200 4200
$Comp
L power:GND #PWR?
U 1 1 5EE2A4C3
P 5500 4650
AR Path="/5EC7245E/5EE2A4C3" Ref="#PWR?"  Part="1" 
AR Path="/5EDA42CC/5EE2A4C3" Ref="#PWR043"  Part="1" 
F 0 "#PWR043" H 5500 4400 50  0001 C CNN
F 1 "GND" H 5505 4477 50  0000 C CNN
F 2 "" H 5500 4650 50  0001 C CNN
F 3 "" H 5500 4650 50  0001 C CNN
	1    5500 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 4650 5500 4150
Wire Wire Line
	5500 4150 5800 4150
NoConn ~ 5800 4050
Text Notes 5100 2700 0    200  ~ 0
Iridium
Text HLabel 4050 3750 0    50   Input ~ 0
IRIDIUM_ON
Text HLabel 4050 3850 0    50   Input ~ 0
IRIDIUM_TX
Text HLabel 4050 3950 0    50   Output ~ 0
IRIDIUM_RX
Text HLabel 4050 4350 0    50   Output ~ 0
IRIDIUM_RING_IND
Text HLabel 4050 4450 0    50   Output ~ 0
IRIDIUM_NET_AVAIL
Wire Wire Line
	4050 4450 5300 4450
Connection ~ 5300 4450
Wire Wire Line
	4050 3950 5200 3950
Wire Wire Line
	4050 3850 5100 3850
Wire Wire Line
	4050 3750 5000 3750
Wire Wire Line
	5800 3950 5200 3950
Wire Wire Line
	5100 3850 5800 3850
Wire Wire Line
	5000 3750 5800 3750
Wire Wire Line
	5300 4450 5800 4450
Wire Wire Line
	5800 4350 4050 4350
$Comp
L PMEL_RF:IRIDIUM_9603N U?
U 1 1 5EE2A4CE
P 6550 3600
AR Path="/5EC7245E/5EE2A4CE" Ref="U?"  Part="1" 
AR Path="/5EDA42CC/5EE2A4CE" Ref="U7"  Part="1" 
F 0 "U7" H 7050 3650 50  0000 C CNN
F 1 "IRIDIUM_9603N" H 6100 3650 50  0000 C CNN
F 2 "PMEL_Modules:Iridium_9603n" H 6550 3600 50  0001 C CNN
F 3 "" H 6550 3600 50  0001 C CNN
F 4 "http://suddendocs.samtec.com/prints/ss4-xx-x.xx-x-d-x-xx-tr-mkt.pdf" H 6550 3600 50  0001 C CNN "Connector Datasheet"
	1    6550 3600
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_Coaxial J?
U 1 1 5F060F58
P 8050 3750
AR Path="/60F8228B/5F060F58" Ref="J?"  Part="1" 
AR Path="/5EE96A32/5F060F58" Ref="J?"  Part="1" 
AR Path="/5EDA42CC/5F060F58" Ref="J22"  Part="1" 
F 0 "J22" H 8150 3725 50  0000 L CNN
F 1 "U.FL" H 8150 3634 50  0000 L CNN
F 2 "Connector_Coaxial:U.FL_Hirose_U.FL-R-SMT-1_Vertical" H 8050 3750 50  0001 C CNN
F 3 " ~" H 8050 3750 50  0001 C CNN
	1    8050 3750
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F0616AC
P 8050 4150
AR Path="/5EC7245E/5F0616AC" Ref="#PWR?"  Part="1" 
AR Path="/5EDA42CC/5F0616AC" Ref="#PWR0101"  Part="1" 
F 0 "#PWR0101" H 8050 3900 50  0001 C CNN
F 1 "GND" H 8055 3977 50  0000 C CNN
F 2 "" H 8050 4150 50  0001 C CNN
F 3 "" H 8050 4150 50  0001 C CNN
	1    8050 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 3950 8050 4150
Text HLabel 8750 3750 2    50   BiDi ~ 0
IRIDIUM_SIGNAL
Wire Wire Line
	8250 3750 8750 3750
Text Notes 8150 4250 0    100  Italic 0
This requires some impedance\n matching calculation for PCB!!!
$EndSCHEMATC
