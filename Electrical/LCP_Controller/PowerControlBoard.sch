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
Text Notes 4800 4050 2    50   ~ 0
COM0
$Comp
L SparkFun-PowerSymbols:VIN #SUPPLY?
U 1 1 5EE23044
P 5800 3650
AR Path="/5EC7245E/5EE23044" Ref="#SUPPLY?"  Part="1" 
AR Path="/5EDA50A5/5EE23044" Ref="#SUPPLY01"  Part="1" 
F 0 "#SUPPLY01" H 5850 3650 45  0001 L BNN
F 1 "VIN" H 5800 3926 45  0000 C CNN
F 2 "External Power Input" H 5800 3831 60  0000 C CNN
F 3 "" H 5800 3650 60  0001 C CNN
	1    5800 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 3800 5800 3650
Wire Wire Line
	5200 3800 5800 3800
$Comp
L SparkFun-PowerSymbols:GND #GND?
U 1 1 5EE2304C
P 5800 4250
AR Path="/5EC7245E/5EE2304C" Ref="#GND?"  Part="1" 
AR Path="/5EDA50A5/5EE2304C" Ref="#GND01"  Part="1" 
F 0 "#GND01" H 5850 4200 45  0001 L BNN
F 1 "GND" H 5800 4080 45  0000 C CNN
F 2 "" H 5800 4150 60  0001 C CNN
F 3 "" H 5800 4150 60  0001 C CNN
	1    5800 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 4200 5800 4250
Wire Wire Line
	5200 4200 5800 4200
Text Notes 4050 3250 0    200  ~ 0
Power Control Board
Text HLabel 6800 4100 2    50   Input ~ 0
PWR_CTRL_SCL
Text HLabel 6800 4000 2    50   BiDi ~ 0
PWR_CTRL_SDA
Wire Wire Line
	5200 4000 6800 4000
Wire Wire Line
	5200 4100 6800 4100
$Comp
L Connector_Generic:Conn_01x05 J1
U 1 1 5EFD9542
P 5000 4000
F 0 "J1" H 4918 3575 50  0000 C CNN
F 1 "Conn_01x05" H 4918 3666 50  0000 C CNN
F 2 "" H 5000 4000 50  0001 C CNN
F 3 "~" H 5000 4000 50  0001 C CNN
	1    5000 4000
	-1   0    0    1   
$EndComp
Text HLabel 6800 3900 2    50   Input ~ 0
PWR_CTRL_ENABLE
Wire Wire Line
	6800 3900 5200 3900
$EndSCHEMATC
