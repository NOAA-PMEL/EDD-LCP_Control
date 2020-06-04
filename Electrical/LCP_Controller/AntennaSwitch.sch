EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 17 20
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
L RF_Switch:AS179-92LF U?
U 1 1 5EEACBA6
P 6000 4100
AR Path="/60F8228B/5EEACBA6" Ref="U?"  Part="1" 
AR Path="/5EE96A32/5EEACBA6" Ref="U15"  Part="1" 
F 0 "U15" H 5800 4400 50  0000 C CNN
F 1 "AS179-92LF" H 5750 3750 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-363_SC-70-6" H 6100 4100 50  0001 C CNN
F 3 "http://www.skyworksinc.com/uploads/documents/AS179_92LF_200176H.pdf" H 6100 4100 50  0001 C CNN
	1    6000 4100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5EEACBAC
P 5250 3900
AR Path="/60F8228B/5EEACBAC" Ref="C?"  Part="1" 
AR Path="/5EE96A32/5EEACBAC" Ref="C45"  Part="1" 
F 0 "C45" H 5342 3946 50  0000 L CNN
F 1 "100pF" H 5342 3855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5250 3900 50  0001 C CNN
F 3 "~" H 5250 3900 50  0001 C CNN
	1    5250 3900
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5EEACBB2
P 5250 4100
AR Path="/60F8228B/5EEACBB2" Ref="C?"  Part="1" 
AR Path="/5EE96A32/5EEACBB2" Ref="C46"  Part="1" 
F 0 "C46" H 5342 4146 50  0000 L CNN
F 1 "100pF" H 5342 4055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5250 4100 50  0001 C CNN
F 3 "~" H 5250 4100 50  0001 C CNN
	1    5250 4100
	0    1    1    0   
$EndComp
Wire Wire Line
	5350 3900 5600 3900
Wire Wire Line
	5350 4100 5600 4100
Text GLabel 6650 4200 2    50   Input ~ 0
GPS_3V3
Wire Wire Line
	6650 4200 6400 4200
$Comp
L Device:C_Small C?
U 1 1 5EEACBBC
P 6700 4000
AR Path="/60F8228B/5EEACBBC" Ref="C?"  Part="1" 
AR Path="/5EE96A32/5EEACBBC" Ref="C47"  Part="1" 
F 0 "C47" H 6792 4046 50  0000 L CNN
F 1 "100pF" H 6792 3955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6700 4000 50  0001 C CNN
F 3 "~" H 6700 4000 50  0001 C CNN
	1    6700 4000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6600 4000 6400 4000
Wire Wire Line
	6800 4000 7050 4000
Wire Wire Line
	7050 4000 7050 3600
Text Notes 7100 2650 2    200  ~ 0
Antenna Switch
Text GLabel 6650 4300 2    50   Input ~ 0
EXT_PWR
Wire Wire Line
	6650 4300 6400 4300
$Comp
L power:GND #PWR?
U 1 1 5EEACBE3
P 5900 4600
AR Path="/60F8228B/5EEACBE3" Ref="#PWR?"  Part="1" 
AR Path="/5EE96A32/5EEACBE3" Ref="#PWR089"  Part="1" 
F 0 "#PWR089" H 5900 4350 50  0001 C CNN
F 1 "GND" H 6000 4500 50  0000 C CNN
F 2 "" H 5900 4600 50  0001 C CNN
F 3 "" H 5900 4600 50  0001 C CNN
	1    5900 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 4500 5900 4600
Text HLabel 4150 3900 0    50   BiDi ~ 0
IRIDIUM_SIGNAL
Text HLabel 4150 4100 0    50   Input ~ 0
GPS_SIGNAL
Wire Wire Line
	4150 4100 5150 4100
Wire Wire Line
	4150 3900 5150 3900
Text HLabel 7450 3600 2    50   BiDi ~ 0
Iridium_GPS_Signal
Wire Wire Line
	7050 3600 7450 3600
$EndSCHEMATC
