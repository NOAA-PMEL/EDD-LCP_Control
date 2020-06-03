EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr B 17000 11000
encoding utf-8
Sheet 2 5
Title "LCP Controller "
Date "5/19/2020"
Rev "0.1"
Comp "NOAA Pacific Marine Environmental Laboratory"
Comment1 "Microcontroller"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Sparkfun-Artemis:Artemis_Module U1
U 1 1 5EC7B8D6
P 7000 3900
F 0 "U1" H 7900 5650 60  0000 C CNN
F 1 "Artemis_Module" H 5900 5600 60  0000 C CNN
F 2 "Artemis:Artemis" H 5650 5500 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/assets/learn_tutorials/9/0/9/Apollo3_Blue_MCU_Data_Sheet_v0_9_1.pdf" H 13750 1150 60  0001 L CNN
	1    7000 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal Y1
U 1 1 5EC7B8DC
P 4300 3850
F 0 "Y1" V 4254 3981 50  0000 L CNN
F 1 "Crystal" V 4345 3981 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_MicroCrystal_CM9V-T1A-2Pin_1.6x1.0mm_HandSoldering" H 4300 3850 50  0001 C CNN
F 3 "~" H 4300 3850 50  0001 C CNN
F 4 "ECS-.327-12.5-12R-C-TR" V 4300 3850 50  0001 C CNN "MPN"
	1    4300 3850
	0    1    1    0   
$EndComp
$Comp
L Device:C C1
U 1 1 5EC7B8E3
P 4000 3700
F 0 "C1" V 3850 3550 50  0000 C CNN
F 1 "15pF" V 4150 3700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4038 3550 50  0001 C CNN
F 3 "~" H 4000 3700 50  0001 C CNN
	1    4000 3700
	0    1    1    0   
$EndComp
$Comp
L Device:C C2
U 1 1 5EC7B8E9
P 4000 4000
F 0 "C2" V 3900 3850 50  0000 C CNN
F 1 "15pF" V 4150 4000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4038 3850 50  0001 C CNN
F 3 "~" H 4000 4000 50  0001 C CNN
	1    4000 4000
	0    1    1    0   
$EndComp
Wire Wire Line
	3850 4000 3600 4000
Wire Wire Line
	3600 4000 3600 4150
Wire Wire Line
	3850 3700 3600 3700
Wire Wire Line
	3600 3700 3600 4000
Connection ~ 3600 4000
Wire Wire Line
	4150 3700 4300 3700
Connection ~ 4300 3700
Wire Wire Line
	4150 4000 4300 4000
Connection ~ 4300 4000
Wire Wire Line
	5450 3800 4700 3800
Wire Wire Line
	4700 3800 4700 4000
Wire Wire Line
	6600 5900 6600 6050
Wire Wire Line
	6600 6050 6700 6050
Wire Wire Line
	6800 6050 6800 6150
Wire Wire Line
	7100 5900 7100 6050
Wire Wire Line
	7100 6050 7000 6050
Connection ~ 6800 6050
Wire Wire Line
	7000 5900 7000 6050
Connection ~ 7000 6050
Wire Wire Line
	7000 6050 6900 6050
Wire Wire Line
	6900 5900 6900 6050
Connection ~ 6900 6050
Wire Wire Line
	6900 6050 6800 6050
Wire Wire Line
	6800 5900 6800 6050
Wire Wire Line
	6700 5900 6700 6050
Connection ~ 6700 6050
Wire Wire Line
	6700 6050 6800 6050
$Comp
L Device:C C3
U 1 1 5EC7B917
P 6050 1700
F 0 "C3" H 6165 1746 50  0000 L CNN
F 1 "10uF" H 6165 1655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6088 1550 50  0001 C CNN
F 3 "~" H 6050 1700 50  0001 C CNN
	1    6050 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5EC7B91D
P 6350 1700
F 0 "C4" H 6465 1746 50  0000 L CNN
F 1 "1uF" H 6465 1655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6388 1550 50  0001 C CNN
F 3 "~" H 6350 1700 50  0001 C CNN
	1    6350 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5EC7B923
P 6650 1700
F 0 "C5" H 6765 1746 50  0000 L CNN
F 1 "0.1uF" H 6765 1655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6688 1550 50  0001 C CNN
F 3 "~" H 6650 1700 50  0001 C CNN
	1    6650 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 1850 6050 1950
Wire Wire Line
	6650 1950 6650 1850
Wire Wire Line
	6350 2000 6350 1950
Wire Wire Line
	6050 1950 6350 1950
Connection ~ 6350 1950
Wire Wire Line
	6350 1950 6650 1950
Wire Wire Line
	6350 1850 6350 1950
Wire Wire Line
	6050 1550 6050 1400
Wire Wire Line
	6050 1400 6350 1400
Wire Wire Line
	6350 1550 6350 1400
Connection ~ 6350 1400
Wire Wire Line
	6350 1400 6650 1400
Wire Wire Line
	6650 1550 6650 1400
Connection ~ 6650 1400
Wire Wire Line
	6350 1150 6350 1400
$Comp
L Connector:Conn_ARM_JTAG_SWD_10 J1
U 1 1 5EC572C3
P 2850 2950
F 0 "J1" H 2407 2996 50  0000 R CNN
F 1 "Conn_ARM_JTAG_SWD_10" H 2407 2905 50  0000 R CNN
F 2 "Connector:Tag-Connect_TC2030-IDC-NL_2x03_P1.27mm_Vertical" H 2850 2950 50  0001 C CNN
F 3 "http://infocenter.arm.com/help/topic/com.arm.doc.ddi0314h/DDI0314H_coresight_components_trm.pdf" V 2500 1700 50  0001 C CNN
	1    2850 2950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR01
U 1 1 5EC572C9
P 2850 2250
F 0 "#PWR01" H 2850 2100 50  0001 C CNN
F 1 "+3.3V" H 2865 2423 50  0000 C CNN
F 2 "" H 2850 2250 50  0001 C CNN
F 3 "" H 2850 2250 50  0001 C CNN
	1    2850 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 2250 2850 2350
Wire Wire Line
	2850 3550 2850 3650
Wire Wire Line
	2750 3550 2750 3650
Wire Wire Line
	2750 3650 2850 3650
Connection ~ 2850 3650
Wire Wire Line
	2850 3650 2850 3800
Wire Wire Line
	5450 4750 3450 4750
Wire Wire Line
	3450 4750 3450 3050
Wire Wire Line
	3450 3050 3350 3050
Wire Wire Line
	5450 3400 3550 3400
Wire Wire Line
	3350 2850 3550 2850
Wire Wire Line
	3350 2950 3500 2950
Wire Wire Line
	3500 3500 5450 3500
Wire Wire Line
	3500 2950 3500 3500
Wire Wire Line
	3550 2850 3550 3400
Wire Wire Line
	3350 2650 5450 2650
Text GLabel 8800 2800 2    50   Input ~ 0
ARTEMIS_D4
Text GLabel 8800 2900 2    50   Input ~ 0
ARTEMIS_D5_SCK(0)
Text GLabel 8800 3000 2    50   Input ~ 0
ARTEMIS_D6_MISO(0)
Text GLabel 8800 3100 2    50   Input ~ 0
ARTEMIS_D7_MOSI(0)
Text GLabel 8800 3200 2    50   Input ~ 0
ARTEMIS_D8_SCL(1)
Text GLabel 8800 3300 2    50   Input ~ 0
ARTEMIS_D9_SDA(1)
Text GLabel 8800 3400 2    50   Input ~ 0
ARTEMIS_D10
Text GLabel 8800 3500 2    50   Input ~ 0
ARTEMIS_D14_TX(1)
Text GLabel 8800 3600 2    50   Input ~ 0
ARTEMIS_D15_RX(1)
Text GLabel 8800 3700 2    50   Input ~ 0
ARTEMIS_D17
Text GLabel 8800 3800 2    50   Input ~ 0
ARTEMIS_D18
Text GLabel 8800 3900 2    50   Input ~ 0
ARTEMIS_D19
Text GLabel 8800 4000 2    50   Input ~ 0
ARTEMIS_D22
Text GLabel 8800 4100 2    50   Input ~ 0
ARTEMIS_D23
Text GLabel 8800 4200 2    50   Input ~ 0
ARTEMIS_D24
Text GLabel 8800 4300 2    50   Input ~ 0
ARTEMIS_D25_SDA(2)
Text GLabel 8800 4400 2    50   Input ~ 0
ARTEMIS_D26
Text GLabel 8800 4500 2    50   Input ~ 0
ARTEMIS_D27_SCL(2)
Text GLabel 8800 4600 2    50   Input ~ 0
ARTEMIS_D28
Text GLabel 8800 4700 2    50   Input ~ 0
ARTEMIS_D36
Text GLabel 8800 4800 2    50   Input ~ 0
ARTEMIS_D37
Text GLabel 8800 4900 2    50   Input ~ 0
ARTEMIS_D38_MOSI(3)
Text GLabel 8800 5000 2    50   Input ~ 0
ARTEMIS_D39_SCL(4)
Text GLabel 8800 5100 2    50   Input ~ 0
ARTEMIS_D40_SDA(4)
Text GLabel 8800 5200 2    50   Input ~ 0
ARTEMIS_D41_BLEIF_IRQ
Text GLabel 8800 5300 2    50   Input ~ 0
ARTEMIS_D42_SCK(3)
Text GLabel 8800 5400 2    50   Input ~ 0
ARTEMIS_D43_MISO(3)
Text GLabel 8800 5500 2    50   Input ~ 0
ARTEMIS_D44
Text GLabel 8800 5600 2    50   Input ~ 0
ARTEMIS_D45
Text GLabel 5200 3100 0    50   Input ~ 0
ARTEMIS_D48_TX(0)
Text GLabel 5200 3200 0    50   Input ~ 0
ARTEMIS_D49_RX(0)
Text GLabel 5300 4050 0    50   Input ~ 0
ARTEMIS_AD11
Text GLabel 5300 4150 0    50   Input ~ 0
ARTEMIS_AD12
Text GLabel 5300 4250 0    50   Input ~ 0
ARTEMIS_AD13
Text GLabel 5300 4350 0    50   Input ~ 0
ARTEMIS_AD16
Text GLabel 5300 4450 0    50   Input ~ 0
ARTEMIS_AD29
Text GLabel 5300 4550 0    50   Input ~ 0
ARTEMIS_AD31
Text GLabel 5300 4650 0    50   Input ~ 0
ARTEMIS_AD31
Text GLabel 5300 4850 0    50   Input ~ 0
ARTEMIS_AD34
Text GLabel 5300 4950 0    50   Input ~ 0
ARTEMIS_AD35
Wire Wire Line
	4700 4000 4300 4000
Wire Wire Line
	5450 4050 5300 4050
Wire Wire Line
	5300 4150 5450 4150
Wire Wire Line
	5450 4250 5300 4250
Wire Wire Line
	5300 4350 5450 4350
Wire Wire Line
	5450 4450 5300 4450
Wire Wire Line
	5300 4550 5450 4550
Wire Wire Line
	5450 4650 5300 4650
Wire Wire Line
	5300 4850 5450 4850
Wire Wire Line
	5450 4950 5300 4950
Wire Wire Line
	5200 3200 5450 3200
Wire Wire Line
	5450 3100 5200 3100
Wire Wire Line
	4300 3700 5450 3700
Wire Wire Line
	8550 2800 8800 2800
Wire Wire Line
	8550 2900 8800 2900
Wire Wire Line
	8550 3000 8800 3000
Wire Wire Line
	8550 3100 8800 3100
Wire Wire Line
	8800 3200 8550 3200
Wire Wire Line
	8550 3300 8800 3300
Wire Wire Line
	8800 3400 8550 3400
Wire Wire Line
	8550 3500 8800 3500
Wire Wire Line
	8800 3600 8550 3600
Wire Wire Line
	8800 3700 8550 3700
Wire Wire Line
	8550 3800 8800 3800
Wire Wire Line
	8800 3900 8550 3900
Wire Wire Line
	8550 4000 8800 4000
Wire Wire Line
	8800 4100 8550 4100
Wire Wire Line
	8800 4200 8550 4200
Wire Wire Line
	8550 4300 8800 4300
Wire Wire Line
	8800 4400 8550 4400
Wire Wire Line
	8800 4500 8550 4500
Wire Wire Line
	8550 4600 8800 4600
Wire Wire Line
	8800 4700 8550 4700
Wire Wire Line
	8550 4800 8800 4800
Wire Wire Line
	8800 4900 8550 4900
Wire Wire Line
	8550 5000 8800 5000
Wire Wire Line
	8800 5100 8550 5100
Wire Wire Line
	8550 5200 8800 5200
Wire Wire Line
	8800 5300 8550 5300
Wire Wire Line
	8550 5400 8800 5400
Wire Wire Line
	8800 5500 8550 5500
Wire Wire Line
	8550 5600 8800 5600
$Comp
L power:+3.3V #PWR04
U 1 1 5ECCB6BB
P 6350 1150
F 0 "#PWR04" H 6350 1000 50  0001 C CNN
F 1 "+3.3V" H 6365 1323 50  0000 C CNN
F 2 "" H 6350 1150 50  0001 C CNN
F 3 "" H 6350 1150 50  0001 C CNN
	1    6350 1150
	1    0    0    -1  
$EndComp
Text GLabel 8800 2700 2    50   Input ~ 0
ARTEMIS_D3
Text GLabel 8800 2600 2    50   Input ~ 0
ARTEMIS_D2
Text GLabel 8800 2500 2    50   Input ~ 0
ARTEMIS_D1
Text GLabel 8800 2400 2    50   Input ~ 0
ARTEMIS_D0
Wire Wire Line
	8800 2400 8550 2400
Wire Wire Line
	8550 2500 8800 2500
Wire Wire Line
	8800 2600 8550 2600
Wire Wire Line
	8550 2700 8800 2700
Wire Wire Line
	6900 1400 6900 2100
Wire Wire Line
	6650 1400 6900 1400
Wire Wire Line
	6800 2150 6800 2100
Wire Wire Line
	6800 2100 6900 2100
Connection ~ 6900 2100
Wire Wire Line
	6900 2100 6900 2150
Text GLabel 5200 2850 0    50   Input ~ 0
ARTEMIS_BOOT
Wire Wire Line
	5450 2850 5200 2850
Text Label 4250 2650 0    50   ~ 0
nRESET
Text Label 5000 3400 0    50   ~ 0
SWDCLK
Text Label 5000 3500 0    50   ~ 0
SWDIO
Text Label 5000 4750 0    50   ~ 0
SWO
Text Label 5000 3700 0    50   ~ 0
XO
Text Label 5000 3800 0    50   ~ 0
XI
$Comp
L power:GND #PWR06
U 1 1 5F1A98F4
P 6800 6150
F 0 "#PWR06" H 6800 5900 50  0001 C CNN
F 1 "GND" H 6805 5977 50  0000 C CNN
F 2 "" H 6800 6150 50  0001 C CNN
F 3 "" H 6800 6150 50  0001 C CNN
	1    6800 6150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5F1A9E13
P 6350 2000
F 0 "#PWR05" H 6350 1750 50  0001 C CNN
F 1 "GND" H 6355 1827 50  0000 C CNN
F 2 "" H 6350 2000 50  0001 C CNN
F 3 "" H 6350 2000 50  0001 C CNN
	1    6350 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5F1AA5CE
P 2850 3800
F 0 "#PWR02" H 2850 3550 50  0001 C CNN
F 1 "GND" H 2855 3627 50  0000 C CNN
F 2 "" H 2850 3800 50  0001 C CNN
F 3 "" H 2850 3800 50  0001 C CNN
	1    2850 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5F1AAAD7
P 3600 4150
F 0 "#PWR03" H 3600 3900 50  0001 C CNN
F 1 "GND" H 3605 3977 50  0000 C CNN
F 2 "" H 3600 4150 50  0001 C CNN
F 3 "" H 3600 4150 50  0001 C CNN
	1    3600 4150
	1    0    0    -1  
$EndComp
Text Notes 1500 1900 0    200  ~ 0
SWD Programmer
$Comp
L Device:LED D1
U 1 1 5EDE0C09
P 12550 2850
F 0 "D1" V 12589 2733 50  0000 R CNN
F 1 "LED" V 12498 2733 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 12550 2850 50  0001 C CNN
F 3 "~" H 12550 2850 50  0001 C CNN
	1    12550 2850
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5EDE1E35
P 13050 2850
F 0 "D2" V 13089 2733 50  0000 R CNN
F 1 "LED" V 12998 2733 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 13050 2850 50  0001 C CNN
F 3 "~" H 13050 2850 50  0001 C CNN
	1    13050 2850
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR07
U 1 1 5EDE26D3
P 13300 1350
F 0 "#PWR07" H 13300 1200 50  0001 C CNN
F 1 "+3.3V" H 13315 1523 50  0000 C CNN
F 2 "" H 13300 1350 50  0001 C CNN
F 3 "" H 13300 1350 50  0001 C CNN
	1    13300 1350
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP1
U 1 1 5EDF7E4D
P 12650 3350
F 0 "TP1" H 12708 3468 50  0000 L CNN
F 1 "DBG1" H 12708 3377 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 12850 3350 50  0001 C CNN
F 3 "~" H 12850 3350 50  0001 C CNN
	1    12650 3350
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP4
U 1 1 5EDF8CDE
P 14150 3350
F 0 "TP4" H 14208 3468 50  0000 L CNN
F 1 "DGB4" H 14208 3377 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 14350 3350 50  0001 C CNN
F 3 "~" H 14350 3350 50  0001 C CNN
	1    14150 3350
	1    0    0    -1  
$EndComp
Text GLabel 14350 3950 2    50   Input ~ 0
ARTEMIS_D2
Text GLabel 14350 4050 2    50   Input ~ 0
ARTEMIS_AD31
Text GLabel 14350 4150 2    50   Input ~ 0
ARTEMIS_AD32
Text GLabel 14350 4250 2    50   Input ~ 0
ARTEMIS_AD29
$Comp
L Jumper:SolderJumper_2_Open JP1
U 1 1 5EE081BA
P 12550 2350
F 0 "JP1" V 12504 2418 50  0000 L CNN
F 1 "JMP_Open" V 12595 2418 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 12550 2350 50  0001 C CNN
F 3 "~" H 12550 2350 50  0001 C CNN
	1    12550 2350
	0    1    1    0   
$EndComp
$Comp
L Jumper:SolderJumper_2_Open JP2
U 1 1 5EE127BD
P 13050 2350
F 0 "JP2" V 13004 2418 50  0000 L CNN
F 1 "JMP_Open" V 13095 2418 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 13050 2350 50  0001 C CNN
F 3 "~" H 13050 2350 50  0001 C CNN
	1    13050 2350
	0    1    1    0   
$EndComp
$Comp
L Jumper:SolderJumper_2_Open JP3
U 1 1 5EE12A2D
P 13550 2350
F 0 "JP3" V 13504 2418 50  0000 L CNN
F 1 "JMP_Open" V 13595 2418 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 13550 2350 50  0001 C CNN
F 3 "~" H 13550 2350 50  0001 C CNN
	1    13550 2350
	0    1    1    0   
$EndComp
$Comp
L Jumper:SolderJumper_2_Open JP4
U 1 1 5EE12F47
P 14050 2350
F 0 "JP4" V 14004 2418 50  0000 L CNN
F 1 "JMP_Open" V 14095 2418 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 14050 2350 50  0001 C CNN
F 3 "~" H 14050 2350 50  0001 C CNN
	1    14050 2350
	0    1    1    0   
$EndComp
$Comp
L Device:R_Pack04 RN1
U 1 1 5EE21489
P 13350 1800
F 0 "RN1" H 13538 1846 50  0000 L CNN
F 1 "R_Pack04" H 13538 1755 50  0000 L CNN
F 2 "Resistor_SMD:R_Array_Concave_4x0402" V 13625 1800 50  0001 C CNN
F 3 "~" H 13350 1800 50  0001 C CNN
F 4 "EXB-28V102JX" H 13350 1800 50  0001 C CNN "MPN"
	1    13350 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	13300 1500 13250 1500
Wire Wire Line
	13150 1500 13150 1600
Wire Wire Line
	13300 1350 13300 1500
Wire Wire Line
	13300 1500 13350 1500
Wire Wire Line
	13450 1500 13450 1600
Connection ~ 13300 1500
Wire Wire Line
	13350 1600 13350 1500
Connection ~ 13350 1500
Wire Wire Line
	13350 1500 13450 1500
Wire Wire Line
	13250 1600 13250 1500
Connection ~ 13250 1500
Wire Wire Line
	13250 1500 13150 1500
Wire Wire Line
	13450 2000 13450 2100
Wire Wire Line
	13450 2100 14050 2100
Wire Wire Line
	14050 2100 14050 2200
Wire Wire Line
	13550 2200 13550 2150
Wire Wire Line
	13550 2150 13350 2150
Wire Wire Line
	13350 2150 13350 2000
Wire Wire Line
	13250 2000 13250 2150
Wire Wire Line
	13250 2150 13050 2150
Wire Wire Line
	13050 2150 13050 2200
Wire Wire Line
	13150 2000 13150 2100
Wire Wire Line
	13150 2100 12550 2100
Wire Wire Line
	12550 2100 12550 2200
$Comp
L Device:LED D3
U 1 1 5EE458CD
P 13550 2850
F 0 "D3" V 13589 2733 50  0000 R CNN
F 1 "LED" V 13498 2733 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 13550 2850 50  0001 C CNN
F 3 "~" H 13550 2850 50  0001 C CNN
	1    13550 2850
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D4
U 1 1 5EE458D3
P 14050 2850
F 0 "D4" V 14089 2733 50  0000 R CNN
F 1 "LED" V 13998 2733 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 14050 2850 50  0001 C CNN
F 3 "~" H 14050 2850 50  0001 C CNN
	1    14050 2850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	14050 2500 14050 2700
Wire Wire Line
	13550 2500 13550 2700
Wire Wire Line
	13050 2500 13050 2700
Wire Wire Line
	12550 2500 12550 2700
Wire Wire Line
	14350 3950 14050 3950
Wire Wire Line
	13550 4050 14350 4050
Wire Wire Line
	14350 4150 13050 4150
Wire Wire Line
	14350 4250 12550 4250
$Comp
L Connector:TestPoint TP2
U 1 1 5EE72B36
P 13150 3350
F 0 "TP2" H 13208 3468 50  0000 L CNN
F 1 "DBG2" H 13208 3377 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 13350 3350 50  0001 C CNN
F 3 "~" H 13350 3350 50  0001 C CNN
	1    13150 3350
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP3
U 1 1 5EE731E3
P 13650 3350
F 0 "TP3" H 13708 3468 50  0000 L CNN
F 1 "DBG3" H 13708 3377 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 13850 3350 50  0001 C CNN
F 3 "~" H 13850 3350 50  0001 C CNN
	1    13650 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	12650 3350 12650 3500
Wire Wire Line
	12650 3500 12550 3500
Connection ~ 12550 3500
Wire Wire Line
	13150 3350 13150 3500
Wire Wire Line
	13150 3500 13050 3500
Connection ~ 13050 3500
Wire Wire Line
	13650 3350 13650 3500
Wire Wire Line
	13650 3500 13550 3500
Connection ~ 13550 3500
Wire Wire Line
	14150 3350 14150 3500
Wire Wire Line
	14150 3500 14050 3500
Connection ~ 14050 3500
Wire Wire Line
	12550 3500 12550 4250
Wire Wire Line
	13050 3500 13050 4150
Wire Wire Line
	13550 3500 13550 4050
Wire Wire Line
	14050 3500 14050 3950
Wire Wire Line
	14050 3000 14050 3500
Wire Wire Line
	13550 3000 13550 3500
Wire Wire Line
	13050 3000 13050 3500
Wire Wire Line
	12550 3000 12550 3500
$Comp
L Connector:TestPoint TP6
U 1 1 5F39CBC9
P 14700 3350
F 0 "TP6" H 14758 3468 50  0000 L CNN
F 1 "DBG1" H 14758 3377 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 14900 3350 50  0001 C CNN
F 3 "~" H 14900 3350 50  0001 C CNN
	1    14700 3350
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP7
U 1 1 5F39D676
P 15000 3350
F 0 "TP7" H 15058 3468 50  0000 L CNN
F 1 "DBG1" H 15058 3377 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 15200 3350 50  0001 C CNN
F 3 "~" H 15200 3350 50  0001 C CNN
	1    15000 3350
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP8
U 1 1 5F39D8E6
P 15300 3350
F 0 "TP8" H 15358 3468 50  0000 L CNN
F 1 "DBG1" H 15358 3377 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 15500 3350 50  0001 C CNN
F 3 "~" H 15500 3350 50  0001 C CNN
	1    15300 3350
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP9
U 1 1 5F39DCFD
P 15600 3350
F 0 "TP9" H 15658 3468 50  0000 L CNN
F 1 "DBG1" H 15658 3377 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5000-5004_Miniature" H 15800 3350 50  0001 C CNN
F 3 "~" H 15800 3350 50  0001 C CNN
	1    15600 3350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5F39E0E9
P 15150 3550
F 0 "#PWR0101" H 15150 3300 50  0001 C CNN
F 1 "GND" H 15155 3377 50  0000 C CNN
F 2 "" H 15150 3550 50  0001 C CNN
F 3 "" H 15150 3550 50  0001 C CNN
	1    15150 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	14700 3350 14700 3450
Wire Wire Line
	14700 3450 15000 3450
Wire Wire Line
	15600 3450 15600 3350
Wire Wire Line
	15150 3550 15150 3450
Connection ~ 15150 3450
Wire Wire Line
	15150 3450 15300 3450
Wire Wire Line
	15300 3350 15300 3450
Connection ~ 15300 3450
Wire Wire Line
	15300 3450 15600 3450
Wire Wire Line
	15000 3350 15000 3450
Connection ~ 15000 3450
Wire Wire Line
	15000 3450 15150 3450
$Comp
L Security:ATECC608A-MAHDA U?
U 1 1 5F46829E
P 14650 7600
AR Path="/60F8228B/5F46829E" Ref="U?"  Part="1" 
AR Path="/5EC7245E/5F46829E" Ref="U10"  Part="1" 
F 0 "U10" H 14421 7646 50  0000 R CNN
F 1 "ATECC608A-MAHDA" H 14421 7555 50  0000 R CNN
F 2 "Package_DFN_QFN:DFN-8-1EP_3x2mm_P0.5mm_EP1.3x1.5mm" H 14650 7600 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATECC608A-CryptoAuthentication-Device-Summary-Data-Sheet-DS40001977B.pdf" H 14800 7850 50  0001 C CNN
	1    14650 7600
	-1   0    0    -1  
$EndComp
$Comp
L Device:C C30
U 1 1 5F46E0EE
P 15150 7150
F 0 "C30" H 15265 7196 50  0000 L CNN
F 1 "0.1uF" H 15265 7105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 15188 7000 50  0001 C CNN
F 3 "~" H 15150 7150 50  0001 C CNN
	1    15150 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	15150 6950 15150 7000
Wire Wire Line
	14650 7300 14650 6950
$Comp
L power:GND #PWR0102
U 1 1 5F479B04
P 15150 7400
F 0 "#PWR0102" H 15150 7150 50  0001 C CNN
F 1 "GND" H 15155 7227 50  0000 C CNN
F 2 "" H 15150 7400 50  0001 C CNN
F 3 "" H 15150 7400 50  0001 C CNN
	1    15150 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	15150 7400 15150 7300
$Comp
L power:GND #PWR0103
U 1 1 5F48009F
P 14650 8000
F 0 "#PWR0103" H 14650 7750 50  0001 C CNN
F 1 "GND" H 14655 7827 50  0000 C CNN
F 2 "" H 14650 8000 50  0001 C CNN
F 3 "" H 14650 8000 50  0001 C CNN
	1    14650 8000
	1    0    0    -1  
$EndComp
Wire Wire Line
	14650 7900 14650 7950
Wire Wire Line
	14750 7900 14750 7950
Wire Wire Line
	14750 7950 14650 7950
Connection ~ 14650 7950
Wire Wire Line
	14650 7950 14650 8000
Text GLabel 13800 7700 0    50   Input ~ 0
ARTEMIS_D8_SCL(1)
Text GLabel 13800 7500 0    50   Input ~ 0
ARTEMIS_D9_SDA(1)
Wire Wire Line
	14350 7500 13900 7500
Wire Wire Line
	13800 7700 14000 7700
Text Notes 15600 5950 2    200  ~ 0
Crypto-Authentication
$Comp
L SparkFun-DiscreteSemi:MOSFET_PCH-DMG2307L Q?
U 1 1 5F49F4AB
P 13650 6700
AR Path="/60F8228B/5F49F4AB" Ref="Q?"  Part="1" 
AR Path="/5EC7245E/5F49F4AB" Ref="Q4"  Part="1" 
F 0 "Q4" H 13815 6605 45  0000 L CNN
F 1 "MOSFET_PCH-DMG2305UX-7" H 13815 6689 45  0000 L CNN
F 2 "SOT23-3" H 13650 6950 20  0001 C CNN
F 3 "" H 13650 6700 50  0001 C CNN
F 4 "TRANS-11308" H 13815 6784 60  0000 L CNN "Field4"
	1    13650 6700
	1    0    0    1   
$EndComp
Wire Wire Line
	13750 6950 13750 6900
$Comp
L power:+3.3V #PWR?
U 1 1 5F49F4B2
P 13750 6300
AR Path="/60F8228B/5F49F4B2" Ref="#PWR?"  Part="1" 
AR Path="/5EC7245E/5F49F4B2" Ref="#PWR0104"  Part="1" 
F 0 "#PWR0104" H 13750 6150 50  0001 C CNN
F 1 "+3.3V" H 13765 6473 50  0000 C CNN
F 2 "" H 13750 6300 50  0001 C CNN
F 3 "" H 13750 6300 50  0001 C CNN
	1    13750 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	13750 6300 13750 6400
$Comp
L Device:R R?
U 1 1 5F49F4B9
P 13500 6400
AR Path="/60F8228B/5F49F4B9" Ref="R?"  Part="1" 
AR Path="/5EC7245E/5F49F4B9" Ref="R10"  Part="1" 
F 0 "R10" V 13293 6400 50  0000 C CNN
F 1 "10k" V 13384 6400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 13430 6400 50  0001 C CNN
F 3 "~" H 13500 6400 50  0001 C CNN
	1    13500 6400
	0    1    1    0   
$EndComp
Wire Wire Line
	13650 6400 13750 6400
Connection ~ 13750 6400
Wire Wire Line
	13750 6400 13750 6500
Wire Wire Line
	13450 6600 13300 6600
Wire Wire Line
	13300 6600 13300 6400
Wire Wire Line
	13300 6400 13350 6400
Text GLabel 13150 6600 0    50   Input ~ 0
ARTEMIS_AD16
Wire Wire Line
	13300 6600 13150 6600
Connection ~ 13300 6600
Wire Wire Line
	13750 6950 13900 6950
Wire Wire Line
	13900 7000 13900 6950
Connection ~ 13900 6950
Wire Wire Line
	13900 6950 14000 6950
Wire Wire Line
	14000 7000 14000 6950
Connection ~ 14000 6950
Wire Wire Line
	14000 6950 14650 6950
$Comp
L Device:R_Pack02 RN?
U 1 1 5F49F4CF
P 13900 7200
AR Path="/5ED049DE/5F49F4CF" Ref="RN?"  Part="1" 
AR Path="/60F8228B/5F49F4CF" Ref="RN?"  Part="1" 
AR Path="/5EC7245E/5F49F4CF" Ref="RN15"  Part="1" 
F 0 "RN15" V 13583 7200 50  0000 C CNN
F 1 "10k(x2)" V 13674 7200 50  0000 C CNN
F 2 "Resistor_SMD:R_Array_Concave_2x0603" V 14075 7200 50  0001 C CNN
F 3 "~" H 13900 7200 50  0001 C CNN
	1    13900 7200
	-1   0    0    1   
$EndComp
Connection ~ 14650 6950
Wire Wire Line
	14650 6950 15150 6950
Wire Wire Line
	13900 7400 13900 7500
Connection ~ 13900 7500
Wire Wire Line
	13900 7500 13800 7500
Wire Wire Line
	14000 7400 14000 7700
Connection ~ 14000 7700
Wire Wire Line
	14000 7700 14350 7700
$EndSCHEMATC