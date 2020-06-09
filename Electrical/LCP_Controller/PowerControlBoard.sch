EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 20
Title "LCP Controller "
Date "2020-06-04"
Rev "0.1"
Comp "NOAA Pacific Marine Environmental Laboratory"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L SparkFun-PowerSymbols:VIN #SUPPLY?
U 1 1 5EE23044
P 4800 3650
AR Path="/5EC7245E/5EE23044" Ref="#SUPPLY?"  Part="1" 
AR Path="/5EDA50A5/5EE23044" Ref="#SUPPLY01"  Part="1" 
F 0 "#SUPPLY01" H 4850 3650 45  0001 L BNN
F 1 "VIN" H 4800 3926 45  0000 C CNN
F 2 "External Power Input" H 4800 3831 60  0000 C CNN
F 3 "" H 4800 3650 60  0001 C CNN
	1    4800 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 3800 4800 3650
Wire Wire Line
	4200 3800 4800 3800
$Comp
L SparkFun-PowerSymbols:GND #GND?
U 1 1 5EE2304C
P 4800 4250
AR Path="/5EC7245E/5EE2304C" Ref="#GND?"  Part="1" 
AR Path="/5EDA50A5/5EE2304C" Ref="#GND01"  Part="1" 
F 0 "#GND01" H 4850 4200 45  0001 L BNN
F 1 "GND" H 4800 4080 45  0000 C CNN
F 2 "" H 4800 4150 60  0001 C CNN
F 3 "" H 4800 4150 60  0001 C CNN
	1    4800 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 4200 4800 4250
Wire Wire Line
	4200 4200 4800 4200
Text Notes 4050 3250 0    200  ~ 0
Power Control Board
Text HLabel 7500 4100 2    50   Input ~ 0
PWR_CTRL_SCL
Text HLabel 7500 4000 2    50   BiDi ~ 0
PWR_CTRL_SDA
Wire Wire Line
	4200 4000 6700 4000
Wire Wire Line
	4200 4100 6800 4100
$Comp
L Connector_Generic:Conn_01x05 J1
U 1 1 5EFD9542
P 4000 4000
F 0 "J1" H 3918 3575 50  0000 C CNN
F 1 "Conn_01x05" H 3918 3666 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical" H 4000 4000 50  0001 C CNN
F 3 "~" H 4000 4000 50  0001 C CNN
F 4 "N/A" H 4000 4000 50  0001 C CNN "MPN"
	1    4000 4000
	-1   0    0    1   
$EndComp
Text HLabel 7500 3900 2    50   Input ~ 0
PWR_CTRL_ENABLE
Wire Wire Line
	7500 3900 6600 3900
Text Notes 3550 4000 0    50   ~ 0
CONTROL
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J?
U 1 1 5EEF1286
P 6200 4700
AR Path="/5ED049DE/5EEF1286" Ref="J?"  Part="1" 
AR Path="/60F8228B/5EEF1286" Ref="J?"  Part="1" 
AR Path="/5EDB548E/5EEF1286" Ref="J?"  Part="1" 
AR Path="/5EDA50A5/5EEF1286" Ref="J30"  Part="1" 
F 0 "J30" H 6300 4900 50  0000 R CNN
F 1 "SALEAE_TEST" H 6450 4400 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 6200 4700 50  0001 C CNN
F 3 "~" H 6200 4700 50  0001 C CNN
F 4 "Power Control" H 6200 4300 50  0000 C CNN "TestPoint"
F 5 "0010897080" H 6200 4700 50  0001 C CNN "MPN"
	1    6200 4700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5600 4900 5600 4950
Connection ~ 5600 4900
Wire Wire Line
	5900 4900 5600 4900
Wire Wire Line
	5600 4800 5600 4900
Connection ~ 5600 4800
Wire Wire Line
	5900 4800 5600 4800
Wire Wire Line
	5600 4700 5600 4800
Wire Wire Line
	5600 4700 5600 4600
Connection ~ 5600 4700
Wire Wire Line
	5900 4700 5600 4700
Wire Wire Line
	5600 4600 5900 4600
$Comp
L power:GND #PWR?
U 1 1 5EEF1297
P 5600 4950
AR Path="/60F8228B/5EEF1297" Ref="#PWR?"  Part="1" 
AR Path="/5EDB548E/5EEF1297" Ref="#PWR?"  Part="1" 
AR Path="/5EDA50A5/5EEF1297" Ref="#PWR0111"  Part="1" 
F 0 "#PWR0111" H 5600 4700 50  0001 C CNN
F 1 "GND" H 5500 4850 50  0000 C CNN
F 2 "" H 5600 4950 50  0001 C CNN
F 3 "" H 5600 4950 50  0001 C CNN
	1    5600 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 4600 6600 4600
Wire Wire Line
	6600 4600 6600 3900
Connection ~ 6600 3900
Wire Wire Line
	6600 3900 4200 3900
Wire Wire Line
	6400 4700 6700 4700
Wire Wire Line
	6700 4700 6700 4000
Connection ~ 6700 4000
Wire Wire Line
	6700 4000 7500 4000
Wire Wire Line
	6800 4800 6800 4100
Wire Wire Line
	6400 4800 6800 4800
Connection ~ 6800 4100
Wire Wire Line
	6800 4100 7500 4100
NoConn ~ 6400 4900
$EndSCHEMATC
