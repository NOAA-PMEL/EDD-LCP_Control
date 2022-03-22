EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "LCP Alkaline Battery Pack"
Date "2020-05-13"
Rev "1"
Comp "NOAA PMEL EDD"
Comment1 "18 Alkaline Zues AA Cells in Series"
Comment2 "SB340 Protection Diode, 18AWG 18\" leads"
Comment3 "Specific pack dimensions on attached pdf doc"
Comment4 ""
$EndDescr
$Comp
L Device:Battery_Cell BAT1
U 1 1 5EBC2C7A
P 2000 3850
F 0 "BAT1" V 1745 3900 50  0000 C CNN
F 1 "ZEUS AA" V 1836 3900 50  0000 C CNN
F 2 "" V 2000 3910 50  0001 C CNN
F 3 "~" V 2000 3910 50  0001 C CNN
	1    2000 3850
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell BAT2
U 1 1 5EBC5444
P 2500 3850
F 0 "BAT2" V 2245 3900 50  0000 C CNN
F 1 "ZEUS AA" V 2336 3900 50  0000 C CNN
F 2 "" V 2500 3910 50  0001 C CNN
F 3 "~" V 2500 3910 50  0001 C CNN
	1    2500 3850
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell BAT17
U 1 1 5EBC621B
P 3450 3850
F 0 "BAT17" V 3195 3900 50  0000 C CNN
F 1 "ZEUS AA" V 3286 3900 50  0000 C CNN
F 2 "" V 3450 3910 50  0001 C CNN
F 3 "~" V 3450 3910 50  0001 C CNN
	1    3450 3850
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell BAT18
U 1 1 5EBC6E0E
P 3950 3850
F 0 "BAT18" V 3695 3900 50  0000 C CNN
F 1 "ZEUS AA" V 3786 3900 50  0000 C CNN
F 2 "" V 3950 3910 50  0001 C CNN
F 3 "~" V 3950 3910 50  0001 C CNN
	1    3950 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	2200 3850 2400 3850
Wire Wire Line
	1600 3850 1900 3850
Wire Wire Line
	3650 3850 3850 3850
$Comp
L Device:D_Schottky D1
U 1 1 5EBCCD50
P 4700 3850
F 0 "D1" H 4700 3634 50  0000 C CNN
F 1 "SB340" H 4700 3725 50  0000 C CNN
F 2 "" H 4700 3850 50  0001 C CNN
F 3 "~" H 4700 3850 50  0001 C CNN
	1    4700 3850
	-1   0    0    1   
$EndComp
Wire Wire Line
	4150 3850 4550 3850
Text Notes 2800 3850 0    157  ~ 0
......
Wire Wire Line
	5300 3950 5550 3950
Wire Notes Line width 10 rgb(255, 0, 27)
	5550 3850 6050 3850
Wire Notes Line width 10 rgb(0, 0, 0)
	5550 3950 6050 3950
Wire Notes Line width 10
	1550 3350 5500 3350
Wire Notes Line width 10
	5500 4350 1550 4350
Text Notes 6500 4050 0    118  ~ 24
20AWG Teflon coated wire\n18" Leads. No connector\nDiodes accessible for testing
Text Notes 5900 4100 0    157  ~ 31
+\n-
Text Notes 1550 4900 0    118  ~ 24
18 Zues AA Alkaline Cells in series\nSB340 protection diode\n
Wire Wire Line
	4850 3850 5550 3850
Wire Wire Line
	1600 3850 1600 4150
Wire Wire Line
	1600 4150 5300 4150
Wire Wire Line
	5300 4150 5300 3950
Wire Notes Line width 10
	5500 3350 5500 4350
Wire Notes Line width 10
	1550 3350 1550 4350
Text Notes 1300 6400 0    157  ~ 31
See Battery Pack Dimension Diagram for construction requirements.
$EndSCHEMATC
