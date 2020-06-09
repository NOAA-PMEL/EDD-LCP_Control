EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr B 17000 11000
encoding utf-8
Sheet 19 20
Title "LCP Controller "
Date "2020-06-04"
Rev "0.1"
Comp "NOAA Pacific Marine Environmental Laboratory"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 2400 6950 0    50   Input ~ 0
COM0_TX
Text HLabel 2400 7050 0    50   Output ~ 0
COM0_RX
Text HLabel 2400 6850 0    50   Input ~ 0
COM0_PWR
Text HLabel 9900 6950 0    50   BiDi ~ 0
COM2_TX(D+)
Text HLabel 9900 7050 0    50   BiDi ~ 0
COM2_RX(D-)
Text HLabel 13650 6950 0    50   BiDi ~ 0
COM3_TX(D+)
Text HLabel 13650 7050 0    50   BiDi ~ 0
COM3_RX(D-)
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J?
U 1 1 5F4FCF41
P 6750 1850
AR Path="/5ED049DE/5F4FCF41" Ref="J?"  Part="1" 
AR Path="/60F8228B/5F4FCF41" Ref="J?"  Part="1" 
AR Path="/5EDA4305/5F4FCF41" Ref="J?"  Part="1" 
AR Path="/5EE033BE/5F4FCF41" Ref="J23"  Part="1" 
F 0 "J23" H 6850 2050 50  0000 R CNN
F 1 "ANALOG_CONN" H 7050 1550 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 6750 1850 50  0001 C CNN
F 3 "~" H 6750 1850 50  0001 C CNN
F 4 "N/A" H 6750 1850 50  0001 C CNN "MPN"
	1    6750 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 1750 7250 1750
Wire Wire Line
	7250 1750 7250 1850
Wire Wire Line
	7050 1850 7250 1850
Connection ~ 7250 1850
Wire Wire Line
	7250 1850 7250 1950
Wire Wire Line
	7050 1950 7250 1950
Connection ~ 7250 1950
Wire Wire Line
	7250 1950 7250 2050
Wire Wire Line
	7050 2050 7250 2050
Connection ~ 7250 2050
Wire Wire Line
	7250 2050 7250 2300
Text HLabel 5900 1750 0    50   Output ~ 0
ADC24_IN0
Text HLabel 5900 1850 0    50   Output ~ 0
ADC24_IN1
Text HLabel 5900 1950 0    50   Output ~ 0
ADC24_IN2
Text HLabel 5900 2050 0    50   Output ~ 0
ADC24_IN3
Wire Wire Line
	5900 2050 6550 2050
Wire Wire Line
	6550 1950 5900 1950
Wire Wire Line
	5900 1850 6550 1850
Wire Wire Line
	6550 1750 5900 1750
Text HLabel 5900 2300 0    50   Output ~ 0
ADC24_GND
Wire Wire Line
	5900 2300 7250 2300
$Comp
L power:GND #PWR?
U 1 1 5F536637
P 10450 2200
AR Path="/60F8228B/5F536637" Ref="#PWR?"  Part="1" 
AR Path="/5EEE892C/5F536637" Ref="#PWR?"  Part="1" 
AR Path="/5EE033BE/5F536637" Ref="#PWR096"  Part="1" 
F 0 "#PWR096" H 10450 1950 50  0001 C CNN
F 1 "GND" H 10550 2100 50  0000 C CNN
F 2 "" H 10450 2200 50  0001 C CNN
F 3 "" H 10450 2200 50  0001 C CNN
	1    10450 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	10600 2100 10450 2100
Wire Wire Line
	10450 2100 10450 2200
$Comp
L Connector_Generic:Conn_01x04 J?
U 1 1 5F536642
P 10800 2000
AR Path="/60F8228B/5F536642" Ref="J?"  Part="1" 
AR Path="/5EEE892C/5F536642" Ref="J?"  Part="1" 
AR Path="/5EE033BE/5F536642" Ref="J25"  Part="1" 
F 0 "J25" H 10718 1575 50  0000 C CNN
F 1 "Conn_01x04" H 10718 1666 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 10800 2000 50  0001 C CNN
F 3 "~" H 10800 2000 50  0001 C CNN
F 4 "N/A" H 10800 2000 50  0001 C CNN "MPN"
	1    10800 2000
	1    0    0    1   
$EndComp
Text HLabel 9650 1900 0    50   Input ~ 0
SPARE_I2C_SCL
Text HLabel 9650 2000 0    50   BiDi ~ 0
SPARE_I2C_SDA
Text HLabel 9650 1800 0    50   BiDi ~ 0
SPARE_I2C_PWR
$Comp
L Connector_Generic:Conn_01x04 J?
U 1 1 5F55B6FB
P 3450 7050
AR Path="/5ED049DE/5F55B6FB" Ref="J?"  Part="1" 
AR Path="/5EE033BE/5F55B6FB" Ref="J22"  Part="1" 
F 0 "J22" H 3450 6700 50  0000 C CNN
F 1 "Conn_01x04" H 3368 6716 50  0001 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3450 7050 50  0001 C CNN
F 3 "~" H 3450 7050 50  0001 C CNN
F 4 "N/A" H 3450 7050 50  0001 C CNN "MPN"
	1    3450 7050
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F55B701
P 3100 7200
AR Path="/5ED049DE/5F55B701" Ref="#PWR?"  Part="1" 
AR Path="/5EE033BE/5F55B701" Ref="#PWR093"  Part="1" 
F 0 "#PWR093" H 3100 6950 50  0001 C CNN
F 1 "GND" H 3200 7100 50  0000 C CNN
F 2 "" H 3100 7200 50  0001 C CNN
F 3 "" H 3100 7200 50  0001 C CNN
	1    3100 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 7200 3100 7150
Wire Wire Line
	3100 7150 3250 7150
Text Notes 3600 7000 0    50   ~ 0
COM0
$Comp
L Connector:Conn_Coaxial J?
U 1 1 5F5682F4
P 3150 1850
AR Path="/60F8228B/5F5682F4" Ref="J?"  Part="1" 
AR Path="/5EE96A32/5F5682F4" Ref="J?"  Part="1" 
AR Path="/5EE033BE/5F5682F4" Ref="J21"  Part="1" 
F 0 "J21" H 3250 1825 50  0000 L CNN
F 1 "Conn_Coaxial" H 3250 1734 50  0000 L CNN
F 2 "Connector_Coaxial:SMA_Amphenol_132134_Vertical" H 3150 1850 50  0001 C CNN
F 3 " ~" H 3150 1850 50  0001 C CNN
F 4 "132134" H 3150 1850 50  0001 C CNN "MPN"
	1    3150 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F5682FA
P 3150 2150
AR Path="/60F8228B/5F5682FA" Ref="#PWR?"  Part="1" 
AR Path="/5EE96A32/5F5682FA" Ref="#PWR?"  Part="1" 
AR Path="/5EE033BE/5F5682FA" Ref="#PWR094"  Part="1" 
F 0 "#PWR094" H 3150 1900 50  0001 C CNN
F 1 "GND" H 3250 2050 50  0000 C CNN
F 2 "" H 3150 2150 50  0001 C CNN
F 3 "" H 3150 2150 50  0001 C CNN
	1    3150 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 2150 3150 2050
Text HLabel 2600 1850 0    50   BiDi ~ 0
Iridium_GPS_Signal
Wire Wire Line
	2950 1850 2600 1850
Wire Notes Line
	4550 750  4550 3100
Wire Notes Line
	1000 3100 1000 750 
Wire Notes Line
	1000 750  4550 750 
Wire Notes Line
	1000 3100 4550 3100
Text Notes 3300 950  2    100  ~ 0
Iridium/GPS Coax
Wire Notes Line
	1050 5800 4700 5800
Wire Notes Line
	4700 5800 4700 8150
Wire Notes Line
	4700 8150 1050 8150
Wire Notes Line
	1050 8150 1050 5800
Text Notes 3100 6000 2    100  ~ 0
COM 0
Wire Wire Line
	2400 7050 3250 7050
Wire Wire Line
	2400 6950 3250 6950
Wire Wire Line
	2400 6850 3250 6850
Text HLabel 6150 6950 0    50   Input ~ 0
COM1_TX
Text HLabel 6150 7050 0    50   Output ~ 0
COM1_RX
Text HLabel 6150 6850 0    50   Input ~ 0
COM1_PWR
$Comp
L Connector_Generic:Conn_01x04 J?
U 1 1 5F7E3E6A
P 7200 7050
AR Path="/5ED049DE/5F7E3E6A" Ref="J?"  Part="1" 
AR Path="/5EE033BE/5F7E3E6A" Ref="J24"  Part="1" 
F 0 "J24" H 7200 6700 50  0000 C CNN
F 1 "Conn_01x04" H 7118 6716 50  0001 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 7200 7050 50  0001 C CNN
F 3 "~" H 7200 7050 50  0001 C CNN
F 4 "N/A" H 7200 7050 50  0001 C CNN "MPN"
	1    7200 7050
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F7E3E70
P 6850 7200
AR Path="/5ED049DE/5F7E3E70" Ref="#PWR?"  Part="1" 
AR Path="/5EE033BE/5F7E3E70" Ref="#PWR095"  Part="1" 
F 0 "#PWR095" H 6850 6950 50  0001 C CNN
F 1 "GND" H 6950 7100 50  0000 C CNN
F 2 "" H 6850 7200 50  0001 C CNN
F 3 "" H 6850 7200 50  0001 C CNN
	1    6850 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 7200 6850 7150
Wire Wire Line
	6850 7150 7000 7150
Text Notes 7350 7000 0    50   ~ 0
COM0
Wire Notes Line
	4800 5800 8450 5800
Wire Notes Line
	8450 5800 8450 8150
Wire Notes Line
	8450 8150 4800 8150
Wire Notes Line
	4800 8150 4800 5800
Text Notes 6850 6000 2    100  ~ 0
COM 1
Wire Wire Line
	6150 7050 7000 7050
Wire Wire Line
	6150 6950 7000 6950
Wire Wire Line
	6150 6850 7000 6850
Text HLabel 9900 6850 0    50   Input ~ 0
COM2_PWR
$Comp
L Connector_Generic:Conn_01x04 J?
U 1 1 5F7E72CD
P 10950 7050
AR Path="/5ED049DE/5F7E72CD" Ref="J?"  Part="1" 
AR Path="/5EE033BE/5F7E72CD" Ref="J26"  Part="1" 
F 0 "J26" H 10950 6700 50  0000 C CNN
F 1 "Conn_01x04" H 10868 6716 50  0001 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 10950 7050 50  0001 C CNN
F 3 "~" H 10950 7050 50  0001 C CNN
F 4 "N/A" H 10950 7050 50  0001 C CNN "MPN"
	1    10950 7050
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F7E72D3
P 10600 7200
AR Path="/5ED049DE/5F7E72D3" Ref="#PWR?"  Part="1" 
AR Path="/5EE033BE/5F7E72D3" Ref="#PWR097"  Part="1" 
F 0 "#PWR097" H 10600 6950 50  0001 C CNN
F 1 "GND" H 10700 7100 50  0000 C CNN
F 2 "" H 10600 7200 50  0001 C CNN
F 3 "" H 10600 7200 50  0001 C CNN
	1    10600 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	10600 7200 10600 7150
Wire Wire Line
	10600 7150 10750 7150
Text Notes 11100 7000 0    50   ~ 0
COM0
Wire Notes Line
	8550 5800 12200 5800
Wire Notes Line
	12200 5800 12200 8150
Wire Notes Line
	12200 8150 8550 8150
Wire Notes Line
	8550 8150 8550 5800
Text Notes 10600 6000 2    100  ~ 0
COM 2
Wire Wire Line
	9900 7050 10750 7050
Wire Wire Line
	9900 6950 10750 6950
Wire Wire Line
	9900 6850 10750 6850
Text HLabel 13650 6850 0    50   Input ~ 0
COM3_PWR
$Comp
L Connector_Generic:Conn_01x04 J?
U 1 1 5F7EAE71
P 14700 7050
AR Path="/5ED049DE/5F7EAE71" Ref="J?"  Part="1" 
AR Path="/5EE033BE/5F7EAE71" Ref="J27"  Part="1" 
F 0 "J27" H 14700 6700 50  0000 C CNN
F 1 "Conn_01x04" H 14618 6716 50  0001 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 14700 7050 50  0001 C CNN
F 3 "~" H 14700 7050 50  0001 C CNN
F 4 "N/A" H 14700 7050 50  0001 C CNN "MPN"
	1    14700 7050
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F7EAE77
P 14350 7200
AR Path="/5ED049DE/5F7EAE77" Ref="#PWR?"  Part="1" 
AR Path="/5EE033BE/5F7EAE77" Ref="#PWR098"  Part="1" 
F 0 "#PWR098" H 14350 6950 50  0001 C CNN
F 1 "GND" H 14450 7100 50  0000 C CNN
F 2 "" H 14350 7200 50  0001 C CNN
F 3 "" H 14350 7200 50  0001 C CNN
	1    14350 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	14350 7200 14350 7150
Wire Wire Line
	14350 7150 14500 7150
Text Notes 14850 7000 0    50   ~ 0
COM0
Wire Notes Line
	12300 5800 15950 5800
Wire Notes Line
	15950 5800 15950 8150
Wire Notes Line
	15950 8150 12300 8150
Wire Notes Line
	12300 8150 12300 5800
Text Notes 14350 6000 2    100  ~ 0
COM 3
Wire Wire Line
	13650 7050 14500 7050
Wire Wire Line
	13650 6950 14500 6950
Wire Wire Line
	13650 6850 14500 6850
Wire Notes Line
	8200 750  8200 3100
Wire Notes Line
	4650 3100 4650 750 
Wire Notes Line
	4650 750  8200 750 
Wire Notes Line
	4650 3100 8200 3100
Text Notes 7300 950  2    100  ~ 0
24-Bit Analog-to-Digital
Wire Notes Line
	11900 750  11900 3100
Wire Notes Line
	8350 3100 8350 750 
Wire Notes Line
	8350 750  11900 750 
Wire Notes Line
	8350 3100 11900 3100
Text Notes 11100 950  2    100  ~ 0
Spare I2C / Pressure Sensor
Wire Wire Line
	9650 1800 10600 1800
Wire Wire Line
	9650 1900 10600 1900
Wire Wire Line
	9650 2000 10600 2000
$EndSCHEMATC
