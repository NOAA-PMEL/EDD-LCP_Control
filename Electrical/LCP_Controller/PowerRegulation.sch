EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 12 21
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 5500 3150 0    200  ~ 0
3V3 Regulator
$Comp
L Regulator_Linear:AP2112K-3.3 U?
U 1 1 5EE48C17
P 5950 3750
AR Path="/5F56D5B9/5EE48C17" Ref="U?"  Part="1" 
AR Path="/5EDA4333/5EE48C17" Ref="U10"  Part="1" 
F 0 "U10" H 5950 4092 50  0000 C CNN
F 1 "AP2112K-3.3" H 5950 4001 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 5950 4075 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/AP2112.pdf" H 5950 3850 50  0001 C CNN
	1    5950 3750
	1    0    0    -1  
$EndComp
$Comp
L Jumper:SolderJumper_2_Bridged JP?
U 1 1 5EE48C1D
P 5750 4200
AR Path="/60F8228B/5EE48C1D" Ref="JP?"  Part="1" 
AR Path="/5F56D5B9/5EE48C1D" Ref="JP?"  Part="1" 
AR Path="/5EDA4333/5EE48C1D" Ref="JP10"  Part="1" 
F 0 "JP10" H 5750 4100 50  0000 C CNN
F 1 "IN_GND_JMPR" H 5800 4000 50  0000 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm" H 5750 4200 50  0001 C CNN
F 3 "~" H 5750 4200 50  0001 C CNN
	1    5750 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 4200 5950 4200
$Comp
L power:GND #PWR?
U 1 1 5EE48C24
P 6250 4300
AR Path="/60F8228B/5EE48C24" Ref="#PWR?"  Part="1" 
AR Path="/5F56D5B9/5EE48C24" Ref="#PWR?"  Part="1" 
AR Path="/5EDA4333/5EE48C24" Ref="#PWR058"  Part="1" 
F 0 "#PWR058" H 6250 4050 50  0001 C CNN
F 1 "GND" H 6350 4200 50  0000 C CNN
F 2 "" H 6250 4300 50  0001 C CNN
F 3 "" H 6250 4300 50  0001 C CNN
	1    6250 4300
	1    0    0    -1  
$EndComp
Connection ~ 5950 4200
Wire Wire Line
	6250 4300 6250 4200
Wire Wire Line
	5950 4050 5950 4200
Wire Wire Line
	5950 4200 6250 4200
$Comp
L Connector:TestPoint TP?
U 1 1 5EE48C2E
P 4600 3500
AR Path="/5F56D5B9/5EE48C2E" Ref="TP?"  Part="1" 
AR Path="/5EDA4333/5EE48C2E" Ref="TP9"  Part="1" 
F 0 "TP9" H 4658 3618 50  0000 L CNN
F 1 "TestPoint" H 4658 3527 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 4800 3500 50  0001 C CNN
F 3 "~" H 4800 3500 50  0001 C CNN
	1    4600 3500
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP?
U 1 1 5EE48C34
P 4650 4200
AR Path="/5F56D5B9/5EE48C34" Ref="TP?"  Part="1" 
AR Path="/5EDA4333/5EE48C34" Ref="TP10"  Part="1" 
F 0 "TP10" H 4708 4318 50  0000 L CNN
F 1 "TestPoint" H 4708 4227 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 4850 4200 50  0001 C CNN
F 3 "~" H 4850 4200 50  0001 C CNN
	1    4650 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 4200 5600 4200
Wire Wire Line
	4600 3500 4600 3650
Wire Wire Line
	4600 3650 5350 3650
$Comp
L power:+3.3V #PWR?
U 1 1 5EE48C3D
P 7150 3550
AR Path="/5F56D5B9/5EE48C3D" Ref="#PWR?"  Part="1" 
AR Path="/5EDA4333/5EE48C3D" Ref="#PWR059"  Part="1" 
F 0 "#PWR059" H 7150 3400 50  0001 C CNN
F 1 "+3.3V" H 7165 3723 50  0000 C CNN
F 2 "" H 7150 3550 50  0001 C CNN
F 3 "" H 7150 3550 50  0001 C CNN
	1    7150 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 3650 6350 3650
Wire Wire Line
	7150 3650 7150 3550
$Comp
L Device:C C?
U 1 1 5EE48C45
P 6350 3850
AR Path="/5F56D5B9/5EE48C45" Ref="C?"  Part="1" 
AR Path="/5EDA4333/5EE48C45" Ref="C28"  Part="1" 
F 0 "C28" H 6465 3896 50  0000 L CNN
F 1 "22uF" H 6465 3805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6388 3700 50  0001 C CNN
F 3 "~" H 6350 3850 50  0001 C CNN
	1    6350 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5EE48C4B
P 6750 3850
AR Path="/5F56D5B9/5EE48C4B" Ref="C?"  Part="1" 
AR Path="/5EDA4333/5EE48C4B" Ref="C29"  Part="1" 
F 0 "C29" H 6865 3896 50  0000 L CNN
F 1 "1uF" H 6865 3805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6788 3700 50  0001 C CNN
F 3 "~" H 6750 3850 50  0001 C CNN
	1    6750 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5EE48C51
P 7150 3850
AR Path="/5F56D5B9/5EE48C51" Ref="C?"  Part="1" 
AR Path="/5EDA4333/5EE48C51" Ref="C30"  Part="1" 
F 0 "C30" H 7265 3896 50  0000 L CNN
F 1 "0.1uF" H 7265 3805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7188 3700 50  0001 C CNN
F 3 "~" H 7150 3850 50  0001 C CNN
	1    7150 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 3700 7150 3650
Connection ~ 7150 3650
Wire Wire Line
	6750 3700 6750 3650
Connection ~ 6750 3650
Wire Wire Line
	6750 3650 7150 3650
Wire Wire Line
	6350 3700 6350 3650
Connection ~ 6350 3650
Wire Wire Line
	6350 3650 6750 3650
Wire Wire Line
	6250 4200 6350 4200
Wire Wire Line
	6350 4200 6350 4000
Connection ~ 6250 4200
Wire Wire Line
	6750 4200 6750 4000
Wire Wire Line
	6350 4200 6750 4200
Connection ~ 6350 4200
Wire Wire Line
	7150 4000 7150 4200
Wire Wire Line
	7150 4200 6750 4200
Connection ~ 6750 4200
$Comp
L Jumper:SolderJumper_3_Bridged12 JP?
U 1 1 5EE48C68
P 5350 3850
AR Path="/5F56D5B9/5EE48C68" Ref="JP?"  Part="1" 
AR Path="/5EDA4333/5EE48C68" Ref="JP9"  Part="1" 
F 0 "JP9" H 5350 3963 50  0000 C CNN
F 1 "SolderJumper_3_Bridged12" H 5350 4054 50  0001 C CNN
F 2 "Jumper:SolderJumper-3_P1.3mm_Bridged12_RoundedPad1.0x1.5mm" H 5350 3850 50  0001 C CNN
F 3 "~" H 5350 3850 50  0001 C CNN
	1    5350 3850
	0    -1   1    0   
$EndComp
Connection ~ 5350 3650
Wire Wire Line
	5350 3650 5650 3650
Wire Wire Line
	5650 3750 5650 3850
Wire Wire Line
	5650 3850 5600 3850
Text GLabel 4800 4600 0    50   Input ~ 0
???_PWR_ENABLE
Wire Wire Line
	4800 4600 5350 4600
Wire Wire Line
	5350 4600 5350 4050
Text Notes 3550 3300 0    100  ~ 0
This regulator is only 6V MAX input!!! Needs to be replaced!!
Wire Wire Line
	4050 3650 4600 3650
Connection ~ 4600 3650
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5EE48C79
P 5600 3950
AR Path="/5F56D5B9/5EE48C79" Ref="#FLG?"  Part="1" 
AR Path="/5EDA4333/5EE48C79" Ref="#FLG03"  Part="1" 
F 0 "#FLG03" H 5600 4025 50  0001 C CNN
F 1 "PWR_FLAG" H 5600 4123 50  0000 C CNN
F 2 "" H 5600 3950 50  0001 C CNN
F 3 "~" H 5600 3950 50  0001 C CNN
	1    5600 3950
	-1   0    0    1   
$EndComp
Wire Wire Line
	5600 3950 5600 3850
Connection ~ 5600 3850
Wire Wire Line
	5600 3850 5500 3850
$Comp
L SparkFun-PowerSymbols:VIN #SUPPLY?
U 1 1 5EE48C82
P 4050 3600
AR Path="/5EC7245E/5EE48C82" Ref="#SUPPLY?"  Part="1" 
AR Path="/5F56D5B9/5EE48C82" Ref="#SUPPLY?"  Part="1" 
AR Path="/5EDA4333/5EE48C82" Ref="#SUPPLY02"  Part="1" 
F 0 "#SUPPLY02" H 4100 3600 45  0001 L BNN
F 1 "VIN" H 4050 3876 45  0000 C CNN
F 2 "External Power Input" H 4050 3781 60  0000 C CNN
F 3 "" H 4050 3600 60  0001 C CNN
	1    4050 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3600 4050 3650
$EndSCHEMATC
