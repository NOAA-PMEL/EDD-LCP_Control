EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 14 21
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
L PMEL_DriverICs:MAX14830 U?
U 1 1 5F170811
P 4900 3950
AR Path="/5ED049DE/5F170811" Ref="U?"  Part="1" 
AR Path="/5EDB3B1A/5F170811" Ref="U?"  Part="1" 
F 0 "U?" H 5450 5900 50  0000 C CNN
F 1 "MAX14830" H 4300 5900 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.15x5.15mm" H 5200 1850 200 0001 C CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/MAX14830.pdf" H 4450 4700 200 0001 C CNN
	1    4900 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 4250 6500 4250
Wire Wire Line
	5800 5150 6450 5150
Text GLabel 2000 4400 0    50   Input ~ 0
ARTEMIS_D43_MISO(3)
Text GLabel 2000 4500 0    50   Input ~ 0
ARTEMIS_D38_MOSI(3)
Text GLabel 2000 4600 0    50   Input ~ 0
ARTEMIS_D42_SCK(3)
Text GLabel 2000 4700 0    50   Input ~ 0
ARTEMIS_D17
Text GLabel 2000 4950 0    50   Input ~ 0
ARTEMIS_D18
Text GLabel 2000 5050 0    50   Input ~ 0
ARTEMIS_D19
$Comp
L power:GND #PWR?
U 1 1 5F17081F
P 4450 6100
AR Path="/5ED049DE/5F17081F" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B1A/5F17081F" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4450 5850 50  0001 C CNN
F 1 "GND" H 4550 6000 50  0000 C CNN
F 2 "" H 4450 6100 50  0001 C CNN
F 3 "" H 4450 6100 50  0001 C CNN
	1    4450 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 3500 4000 3500
Wire Wire Line
	2250 3600 4000 3600
Wire Wire Line
	5800 4050 6300 4050
Wire Wire Line
	5800 4150 6400 4150
Wire Wire Line
	5800 4950 6250 4950
Wire Wire Line
	5800 5050 6350 5050
Wire Wire Line
	6300 4000 6300 4050
Connection ~ 6300 4050
Wire Wire Line
	6400 4000 6400 4150
Connection ~ 6400 4150
Wire Wire Line
	6500 4000 6500 4250
Connection ~ 6500 4250
Wire Wire Line
	6250 5300 6250 4950
Connection ~ 6250 4950
Wire Wire Line
	6350 5300 6350 5050
Connection ~ 6350 5050
Wire Wire Line
	6450 5300 6450 5150
Connection ~ 6450 5150
$Comp
L power:GND #PWR?
U 1 1 5F17083D
P 6250 5900
AR Path="/5ED049DE/5F17083D" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B1A/5F17083D" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6250 5650 50  0001 C CNN
F 1 "GND" H 6350 5800 50  0000 C CNN
F 2 "" H 6250 5900 50  0001 C CNN
F 3 "" H 6250 5900 50  0001 C CNN
	1    6250 5900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F170843
P 7000 3500
AR Path="/5ED049DE/5F170843" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B1A/5F170843" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 7000 3250 50  0001 C CNN
F 1 "GND" H 7100 3400 50  0000 C CNN
F 2 "" H 7000 3500 50  0001 C CNN
F 3 "" H 7000 3500 50  0001 C CNN
	1    7000 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 3500 7000 3400
Wire Wire Line
	6300 3400 6300 3500
Wire Wire Line
	6400 3500 6400 3400
Connection ~ 6400 3400
Wire Wire Line
	6400 3400 6300 3400
Wire Wire Line
	6500 3500 6500 3400
Wire Wire Line
	6500 3400 6400 3400
Connection ~ 6500 3400
Wire Wire Line
	7000 3400 6600 3400
Wire Wire Line
	6600 3400 6500 3400
Connection ~ 6600 3400
Wire Wire Line
	6600 3500 6600 3400
$Comp
L power:GND #PWR?
U 1 1 5F170855
P 7150 1500
AR Path="/5ED049DE/5F170855" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B1A/5F170855" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 7150 1250 50  0001 C CNN
F 1 "GND" H 7250 1400 50  0000 C CNN
F 2 "" H 7150 1500 50  0001 C CNN
F 3 "" H 7150 1500 50  0001 C CNN
	1    7150 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 1500 7150 1250
Wire Wire Line
	2550 1600 2550 1750
Wire Wire Line
	2650 1600 2550 1600
Connection ~ 2550 1600
Wire Wire Line
	2300 1600 2550 1600
Wire Wire Line
	2650 1750 2650 1600
Connection ~ 2650 1600
Wire Wire Line
	2650 1600 2750 1600
Wire Wire Line
	2750 1750 2750 1600
Connection ~ 2750 1600
Wire Wire Line
	2750 1600 2850 1600
$Comp
L power:GND #PWR?
U 1 1 5F170866
P 2300 1750
AR Path="/5ED049DE/5F170866" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B1A/5F170866" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2300 1500 50  0001 C CNN
F 1 "GND" H 2400 1650 50  0000 C CNN
F 2 "" H 2300 1750 50  0001 C CNN
F 3 "" H 2300 1750 50  0001 C CNN
	1    2300 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 1750 2300 1600
Wire Wire Line
	4000 2500 3850 2500
Wire Wire Line
	3250 2500 3250 1300
Wire Wire Line
	5100 1300 5100 1950
Wire Wire Line
	4650 1950 4650 1300
Connection ~ 4650 1300
Wire Wire Line
	4650 1300 4850 1300
Wire Wire Line
	4850 1950 4850 1300
Connection ~ 4850 1300
Wire Wire Line
	4850 1300 5100 1300
$Comp
L Device:C_Small C?
U 1 1 5F170876
P 3450 1550
AR Path="/5ED049DE/5F170876" Ref="C?"  Part="1" 
AR Path="/5EDB3B1A/5F170876" Ref="C?"  Part="1" 
F 0 "C?" H 3542 1596 50  0000 L CNN
F 1 "0.1uF" H 3542 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3450 1550 50  0001 C CNN
F 3 "~" H 3450 1550 50  0001 C CNN
	1    3450 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5F17087C
P 3900 1550
AR Path="/5ED049DE/5F17087C" Ref="C?"  Part="1" 
AR Path="/5EDB3B1A/5F17087C" Ref="C?"  Part="1" 
F 0 "C?" H 3992 1596 50  0000 L CNN
F 1 "0.1uF" H 3992 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3900 1550 50  0001 C CNN
F 3 "~" H 3900 1550 50  0001 C CNN
	1    3900 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5F170882
P 5550 1550
AR Path="/5ED049DE/5F170882" Ref="C?"  Part="1" 
AR Path="/5EDB3B1A/5F170882" Ref="C?"  Part="1" 
F 0 "C?" H 5642 1596 50  0000 L CNN
F 1 "1uF" H 5642 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5550 1550 50  0001 C CNN
F 3 "~" H 5550 1550 50  0001 C CNN
	1    5550 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5F170888
P 4300 1550
AR Path="/5ED049DE/5F170888" Ref="C?"  Part="1" 
AR Path="/5EDB3B1A/5F170888" Ref="C?"  Part="1" 
F 0 "C?" H 4392 1596 50  0000 L CNN
F 1 "0.1uF" H 4392 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4300 1550 50  0001 C CNN
F 3 "~" H 4300 1550 50  0001 C CNN
	1    4300 1550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F17088E
P 3900 1800
AR Path="/5ED049DE/5F17088E" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B1A/5F17088E" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3900 1550 50  0001 C CNN
F 1 "GND" H 4000 1700 50  0000 C CNN
F 2 "" H 3900 1800 50  0001 C CNN
F 3 "" H 3900 1800 50  0001 C CNN
	1    3900 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2250 3850 2250
Wire Wire Line
	3850 2250 3850 2500
Connection ~ 3850 2500
Wire Wire Line
	3850 2500 3250 2500
Wire Wire Line
	3250 1300 3450 1300
Connection ~ 3450 1300
Wire Wire Line
	3450 1300 3450 1450
Wire Wire Line
	3450 1300 3900 1300
Connection ~ 3900 1300
Wire Wire Line
	3900 1450 3900 1300
Wire Wire Line
	4300 1300 4650 1300
Wire Wire Line
	3900 1300 4300 1300
Connection ~ 4300 1300
Wire Wire Line
	4300 1450 4300 1300
Wire Wire Line
	4300 1650 4300 1700
Wire Wire Line
	4300 1700 3900 1700
Wire Wire Line
	3900 1700 3900 1650
Connection ~ 3900 1700
Wire Wire Line
	3900 1700 3900 1800
Wire Wire Line
	3450 1650 3450 1700
Wire Wire Line
	3450 1700 3900 1700
$Comp
L power:GND #PWR?
U 1 1 5F1708A9
P 5550 1750
AR Path="/5ED049DE/5F1708A9" Ref="#PWR?"  Part="1" 
AR Path="/5EDB3B1A/5F1708A9" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 5550 1500 50  0001 C CNN
F 1 "GND" H 5650 1650 50  0000 C CNN
F 2 "" H 5550 1750 50  0001 C CNN
F 3 "" H 5550 1750 50  0001 C CNN
	1    5550 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 1750 5550 1650
Wire Wire Line
	5300 1950 5300 1300
Wire Wire Line
	5300 1300 5550 1300
Wire Wire Line
	5550 1300 5550 1450
Wire Wire Line
	5100 5950 5100 6050
Wire Wire Line
	5100 6050 5200 6050
Wire Wire Line
	5200 6050 5200 5950
$Comp
L Device:Crystal Y?
U 1 1 5F1708B6
P 3200 4750
AR Path="/5ED049DE/5F1708B6" Ref="Y?"  Part="1" 
AR Path="/5EDB3B1A/5F1708B6" Ref="Y?"  Part="1" 
F 0 "Y?" V 3154 4881 50  0000 L CNN
F 1 "4MHz" V 3245 4881 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_HC49-SD_HandSoldering" H 3200 4750 50  0001 C CNN
F 3 "~" H 3200 4750 50  0001 C CNN
	1    3200 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	3200 4600 3500 4600
Wire Wire Line
	3500 4600 3500 4650
Wire Wire Line
	3500 4650 4000 4650
Wire Wire Line
	4000 4900 3200 4900
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J?
U 1 1 5F1708CD
P 6350 5500
AR Path="/5ED049DE/5F1708CD" Ref="J?"  Part="1" 
AR Path="/5EDB3B1A/5F1708CD" Ref="J?"  Part="1" 
F 0 "J?" V 6300 5100 50  0000 L CNN
F 1 "Saleae Test" V 6400 4800 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 6350 5500 50  0001 C CNN
F 3 "~" H 6350 5500 50  0001 C CNN
F 4 "COM3 UART Test Points" V 6500 4800 50  0000 C CNN "Field4"
	1    6350 5500
	0    -1   1    0   
$EndComp
Wire Wire Line
	6250 5800 6350 5800
Connection ~ 6350 5800
Wire Wire Line
	6350 5800 6450 5800
Connection ~ 6450 5800
Wire Wire Line
	6450 5800 6550 5800
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J?
U 1 1 5F1708D9
P 6400 3800
AR Path="/5ED049DE/5F1708D9" Ref="J?"  Part="1" 
AR Path="/5EDB3B1A/5F1708D9" Ref="J?"  Part="1" 
F 0 "J?" V 6496 3512 50  0000 R CNN
F 1 "Saleae Test" V 6405 3512 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 6400 3800 50  0001 C CNN
F 3 "~" H 6400 3800 50  0001 C CNN
F 4 "COM2 UART Test Points" V 6300 3050 50  0000 C CNN "Field4"
	1    6400 3800
	0    -1   -1   0   
$EndComp
Connection ~ 6250 5800
Wire Wire Line
	6250 5800 6250 5900
Wire Wire Line
	2250 3250 2850 3250
Wire Wire Line
	2850 3250 4000 3250
Connection ~ 2850 3250
Wire Wire Line
	2850 2250 2850 3250
Wire Wire Line
	2250 3150 2750 3150
Wire Wire Line
	2750 3150 4000 3150
Connection ~ 2750 3150
Wire Wire Line
	2750 2250 2750 3150
Wire Wire Line
	2250 3050 2650 3050
Wire Wire Line
	2650 3050 4000 3050
Connection ~ 2650 3050
Wire Wire Line
	2650 2250 2650 3050
Wire Wire Line
	2250 2950 2550 2950
Wire Wire Line
	2550 2950 4000 2950
Connection ~ 2550 2950
Wire Wire Line
	2550 2250 2550 2950
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J?
U 1 1 5F1708F2
P 2650 2050
AR Path="/5ED049DE/5F1708F2" Ref="J?"  Part="1" 
AR Path="/5EDB3B1A/5F1708F2" Ref="J?"  Part="1" 
F 0 "J?" V 2650 2350 50  0000 R CNN
F 1 "Saleae Test" V 2550 2700 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 2650 2050 50  0001 C CNN
F 3 "~" H 2650 2050 50  0001 C CNN
F 4 "SPI to UART Test Points" V 2450 2700 50  0000 C CNN "Field4"
	1    2650 2050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2850 1600 2850 1750
$Comp
L Jumper:SolderJumper_2_Bridged JP?
U 1 1 5F1708F9
P 4900 6100
AR Path="/5ED049DE/5F1708F9" Ref="JP?"  Part="1" 
AR Path="/5EDB3B1A/5F1708F9" Ref="JP?"  Part="1" 
F 0 "JP?" H 4900 6000 50  0000 C CNN
F 1 "S2U_GND_JMP" H 4900 5900 50  0000 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged2Bar_Pad1.0x1.5mm" H 4900 6100 50  0001 C CNN
F 3 "~" H 4900 6100 50  0001 C CNN
	1    4900 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 6100 4650 6100
Wire Wire Line
	4800 5950 4800 6000
Wire Wire Line
	4800 6000 4650 6000
Wire Wire Line
	4650 6000 4650 6100
Connection ~ 4650 6100
Wire Wire Line
	4650 6100 4750 6100
Wire Wire Line
	5050 6100 5100 6100
Connection ~ 5100 6050
Wire Wire Line
	5100 6100 5100 6050
Wire Wire Line
	7150 1250 6850 1250
Connection ~ 6850 1250
Wire Wire Line
	6850 1350 6850 1250
Wire Wire Line
	5800 3250 6850 3250
Connection ~ 6850 3250
Wire Wire Line
	6850 1850 6850 3250
Wire Wire Line
	6850 1250 6750 1250
Connection ~ 6750 1250
Wire Wire Line
	6750 1250 6750 1350
Wire Wire Line
	5800 3150 6750 3150
Connection ~ 6750 3150
Wire Wire Line
	6750 1850 6750 3150
Wire Wire Line
	6650 1250 6550 1250
Wire Wire Line
	6750 1250 6650 1250
Connection ~ 6650 1250
Wire Wire Line
	6650 1350 6650 1250
Wire Wire Line
	5800 2350 6650 2350
Connection ~ 6650 2350
Wire Wire Line
	6650 1850 6650 2350
Wire Wire Line
	6550 1250 6550 1350
Wire Wire Line
	5800 2250 6550 2250
Connection ~ 6550 2250
Wire Wire Line
	6550 1850 6550 2250
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J?
U 1 1 5F170924
P 6650 1650
AR Path="/5ED049DE/5F170924" Ref="J?"  Part="1" 
AR Path="/5EDB3B1A/5F170924" Ref="J?"  Part="1" 
F 0 "J?" V 6550 1400 50  0000 R CNN
F 1 "Saleae Test" V 6450 1400 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 6650 1650 50  0001 C CNN
F 3 "~" H 6650 1650 50  0001 C CNN
F 4 "COM0/COM1 UART Test Points" V 6350 850 50  0000 C CNN "Field4"
	1    6650 1650
	0    -1   -1   0   
$EndComp
NoConn ~ 6550 5300
NoConn ~ 6600 4000
NoConn ~ 5800 2450
NoConn ~ 5800 2550
NoConn ~ 5800 2650
NoConn ~ 5800 3350
NoConn ~ 5800 3450
NoConn ~ 5800 3550
NoConn ~ 5800 3650
NoConn ~ 5800 3750
NoConn ~ 5800 3850
NoConn ~ 5800 4350
NoConn ~ 5800 4450
NoConn ~ 5800 5250
NoConn ~ 5800 5350
NoConn ~ 5800 5450
NoConn ~ 5800 5550
NoConn ~ 5800 5650
Text Notes 4650 900  0    100  ~ 0
SPI to UART Converter
Text HLabel 10350 2250 2    50   Output ~ 0
COM0_UART_TX
Text HLabel 10350 2350 2    50   Input ~ 0
COM0_UART_RX
Text HLabel 10350 3150 2    50   Output ~ 0
COM1_UART_TX
Text HLabel 10350 3250 2    50   Input ~ 0
COM1_UART_RX
Text HLabel 10350 4050 2    50   Output ~ 0
COM2_UART_TX
Text HLabel 10350 4150 2    50   Input ~ 0
COM2_UART_RX
Text HLabel 10350 4950 2    50   Output ~ 0
COM3_UART_TX
Text HLabel 10350 5050 2    50   Input ~ 0
COM3_UART_RX
Text HLabel 10350 4250 2    50   Output ~ 0
~COM2_UART_RTS
Text HLabel 10350 5150 2    50   Output ~ 0
~COM3_UART_RTS
Wire Wire Line
	6250 4950 10350 4950
Wire Wire Line
	6350 5050 10350 5050
Wire Wire Line
	6450 5150 10350 5150
Wire Wire Line
	6300 4050 10350 4050
Wire Wire Line
	6400 4150 10350 4150
Wire Wire Line
	6500 4250 10350 4250
Wire Wire Line
	6850 3250 10350 3250
Wire Wire Line
	6750 3150 10350 3150
Wire Wire Line
	6650 2350 10350 2350
Wire Wire Line
	6550 2250 10350 2250
Text HLabel 7250 2750 2    50   Output ~ 0
COM01_ON
Text HLabel 7250 2850 2    50   Output ~ 0
COM01_OFF
Text HLabel 7250 2950 2    50   Output ~ 0
~COM01_INV
Text HLabel 7150 4550 2    50   Output ~ 0
COM23_ON
Text HLabel 7150 4650 2    50   Output ~ 0
COM23_OFF
Text HLabel 7150 4750 2    50   Output ~ 0
~COM23_INV
Wire Wire Line
	5800 2750 7250 2750
Wire Wire Line
	5800 2850 7250 2850
Wire Wire Line
	5800 2950 7250 2950
Wire Wire Line
	5800 4550 7150 4550
Wire Wire Line
	5800 4650 7150 4650
Wire Wire Line
	5800 4750 7150 4750
Text HLabel 1550 2950 0    50   Output ~ 0
S2U_MISO
Text HLabel 1550 3050 0    50   Input ~ 0
S2U_MOSI
Text HLabel 1550 3150 0    50   Input ~ 0
S2U_SCK
Text HLabel 1550 3250 0    50   Input ~ 0
~S2U_CS
Text HLabel 1550 3500 0    50   Output ~ 0
~S2U_IRQ
Text HLabel 1550 3600 0    50   Input ~ 0
~S2U_RST
Text HLabel 1850 6100 0    50   Input ~ 0
COM0_POWER_ON
Text HLabel 1850 6200 0    50   Input ~ 0
COM1_POWER_ON
Text HLabel 1850 6300 0    50   Input ~ 0
COM2_POWER_ON
Text HLabel 1850 6400 0    50   Input ~ 0
COM3_POWER_ON
Text HLabel 10300 6100 2    50   Output ~ 0
COM0_ON
Text HLabel 10300 6200 2    50   Output ~ 0
COM1_ON
Text HLabel 10300 6300 2    50   Output ~ 0
COM2_ON
Text HLabel 10300 6400 2    50   Output ~ 0
COM3_ON
Wire Wire Line
	1850 6100 3300 6100
Wire Wire Line
	3300 6100 3300 6650
Wire Wire Line
	3300 6650 6600 6650
Wire Wire Line
	6600 6650 6600 6100
Wire Wire Line
	6600 6100 10300 6100
Wire Wire Line
	1850 6200 3250 6200
Wire Wire Line
	3250 6200 3250 6700
Wire Wire Line
	3250 6700 6650 6700
Wire Wire Line
	6650 6700 6650 6200
Wire Wire Line
	6650 6200 10300 6200
Wire Wire Line
	10300 6300 6700 6300
Wire Wire Line
	6700 6300 6700 6750
Wire Wire Line
	6700 6750 3200 6750
Wire Wire Line
	3200 6750 3200 6300
Wire Wire Line
	3200 6300 1850 6300
Wire Wire Line
	1850 6400 3150 6400
Wire Wire Line
	3150 6400 3150 6800
Wire Wire Line
	3150 6800 6750 6800
Wire Wire Line
	6750 6800 6750 6400
Wire Wire Line
	6750 6400 10300 6400
$EndSCHEMATC
