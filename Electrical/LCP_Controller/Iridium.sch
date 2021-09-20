EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 14 19
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
L power:GND #PWR?
U 1 1 5EE2A477
P 6550 5000
AR Path="/5EC7245E/5EE2A477" Ref="#PWR?"  Part="1" 
AR Path="/5EDA42CC/5EE2A477" Ref="#PWR081"  Part="1" 
F 0 "#PWR081" H 6550 4750 50  0001 C CNN
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
NoConn ~ 7200 3900
NoConn ~ 7200 4000
NoConn ~ 7200 4100
NoConn ~ 7200 4200
$Comp
L power:GND #PWR?
U 1 1 5EE2A4C3
P 5500 4650
AR Path="/5EC7245E/5EE2A4C3" Ref="#PWR?"  Part="1" 
AR Path="/5EDA42CC/5EE2A4C3" Ref="#PWR080"  Part="1" 
F 0 "#PWR080" H 5500 4400 50  0001 C CNN
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
Text Notes 4900 2600 0    200  ~ 0
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
	5800 4350 4050 4350
$Comp
L PMEL_RF:IRIDIUM_9603N U?
U 1 1 5EE2A4CE
P 6550 3600
AR Path="/5EC7245E/5EE2A4CE" Ref="U?"  Part="1" 
AR Path="/5EDA42CC/5EE2A4CE" Ref="U14"  Part="1" 
F 0 "U14" H 7050 3650 50  0000 C CNN
F 1 "IRIDIUM_9603N" H 6100 3650 50  0000 C CNN
F 2 "PMEL_Modules:Iridium_9603n" H 6550 3600 50  0001 C CNN
F 3 "" H 6550 3600 50  0001 C CNN
F 4 "http://suddendocs.samtec.com/prints/ss4-xx-x.xx-x-d-x-xx-tr-mkt.pdf" H 6550 3600 50  0001 C CNN "Connector Datasheet"
F 5 "SS4-10-3.00-L-D-K-TR" H 6550 3600 50  0001 C CNN "Connector PN"
F 6 "SS4-10-3.00-L-D-K-TR" H 6550 3600 50  0001 C CNN "MPN"
	1    6550 3600
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_Coaxial J?
U 1 1 5F060F58
P 8050 3750
AR Path="/60F8228B/5F060F58" Ref="J?"  Part="1" 
AR Path="/5EE96A32/5F060F58" Ref="J?"  Part="1" 
AR Path="/5EDA42CC/5F060F58" Ref="J14"  Part="1" 
F 0 "J14" H 8150 3725 50  0000 L CNN
F 1 "U.FL" H 8150 3634 50  0000 L CNN
F 2 "Connector_Coaxial:U.FL_Hirose_U.FL-R-SMT-1_Vertical" H 8050 3750 50  0001 C CNN
F 3 " ~" H 8050 3750 50  0001 C CNN
F 4 "U.FL-R-SMT-1(10)" H 8050 3750 50  0001 C CNN "MPN"
	1    8050 3750
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F0616AC
P 8050 4150
AR Path="/5EC7245E/5F0616AC" Ref="#PWR?"  Part="1" 
AR Path="/5EDA42CC/5F0616AC" Ref="#PWR082"  Part="1" 
F 0 "#PWR082" H 8050 3900 50  0001 C CNN
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
Text Notes 3150 5900 0    100  Italic 20
Alex: Please check pinout & footprint on U14\n9603N Brochure with layout: \nhttps://www.beamcommunications.com/document/37-brochure-iridium-9603-9603n-transceiver\nHad to move connector on footprint 3mm
$Comp
L Connector:TestPoint TP54
U 1 1 5EE03645
P 6450 3300
F 0 "TP54" H 6508 3463 50  0000 L CNN
F 1 "5000" H 6508 3372 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 6650 3300 50  0001 C CNN
F 3 "~" H 6650 3300 50  0001 C CNN
F 4 "EXT_PWR" H 6508 3281 50  0000 L CNN "TestPoint"
F 5 "" H 6450 3300 50  0001 C CNN "MPN"
	1    6450 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 3300 6450 3400
Wire Wire Line
	4050 3750 5800 3750
Wire Wire Line
	4050 3850 5800 3850
Wire Wire Line
	4050 3950 5800 3950
Wire Wire Line
	4050 4450 5800 4450
$EndSCHEMATC
