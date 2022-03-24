EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "LFS Conductivity Sensor"
Date "2020-04-15"
Rev "1"
Comp "NOAA PMEL/EDD"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Conductivity_Sensor-rescue:INA121U-TI_INA121 U2
U 1 1 5E8E13A4
P 3450 2900
F 0 "U2" H 2850 2250 60  0000 C CNN
F 1 "INA121U" H 2700 2150 60  0000 C CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 3450 2840 60  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/ina121" H 3450 2900 60  0001 C CNN
F 4 "INA121U/2K5" H 3450 2900 50  0001 C CNN "Manu.PN"
F 5 "INA121U/2K5" H 3450 2900 50  0001 C CNN "DigikeyPN"
	1    3450 2900
	-1   0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:LMH6702MF-Amplifier_Operational U3
U 1 1 5E8E5C73
P 5950 5050
F 0 "U3" H 6050 4850 50  0000 L CNN
F 1 "LMP7701" H 6050 4750 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5_HandSoldering" H 6100 4950 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/lmp7701.pdf" H 5950 5055 50  0001 C CNN
F 4 "LMP7701MF/NOPB" H 5950 5050 50  0001 C CNN "DigikeyPN"
	1    5950 5050
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:GND-power #PWR0101
U 1 1 5E8E776D
P 3250 3600
F 0 "#PWR0101" H 3250 3350 50  0001 C CNN
F 1 "GND" H 3255 3427 50  0000 C CNN
F 2 "" H 3250 3600 50  0001 C CNN
F 3 "" H 3250 3600 50  0001 C CNN
	1    3250 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 3300 3250 3600
$Comp
L Conductivity_Sensor-rescue:LMH6702MF-Amplifier_Operational U4
U 1 1 5E8F8FD8
P 7600 4950
F 0 "U4" H 7700 5250 50  0000 C CNN
F 1 "LMP7701" H 7800 5350 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5_HandSoldering" H 7750 4850 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/lmp7701.pdf" H 7600 4955 50  0001 C CNN
F 4 "LMP7701MF/NOPB" H 7600 4950 50  0001 C CNN "DigikeyPN"
	1    7600 4950
	1    0    0    1   
$EndComp
$Comp
L Conductivity_Sensor-rescue:LMH6702MF-Amplifier_Operational U5
U 1 1 5E8F936A
P 9450 4850
F 0 "U5" H 9500 4600 50  0000 C CNN
F 1 "LMP7701" H 9600 4700 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5_HandSoldering" H 9600 4750 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/lmp7701.pdf" H 9450 4855 50  0001 C CNN
F 4 "LMP7701MF/NOPB" H 9450 4850 50  0001 C CNN "DigikeyPN"
	1    9450 4850
	1    0    0    1   
$EndComp
$Comp
L Conductivity_Sensor-rescue:OPA188xxD-Amplifier_Operational U1
U 1 1 5E8F9684
P 2450 1500
F 0 "U1" H 2600 1350 50  0000 L CNN
F 1 "OP07" H 2600 1250 50  0000 L CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 2600 1400 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/op07c.pdf" H 2450 1505 50  0001 C CNN
F 4 "OP07CDR" H 2450 1500 50  0001 C CNN "Manu.PN"
F 5 "OP07CDR" H 2450 1500 50  0001 C CNN "DigikeyPN"
	1    2450 1500
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:VSS-power #PWR0102
U 1 1 5E8F9CC6
P 7500 4200
F 0 "#PWR0102" H 7500 4050 50  0001 C CNN
F 1 "VSS" H 7517 4373 50  0000 C CNN
F 2 "" H 7500 4200 50  0001 C CNN
F 3 "" H 7500 4200 50  0001 C CNN
	1    7500 4200
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:VSS-power #PWR0103
U 1 1 5E8FAD46
P 9350 4100
F 0 "#PWR0103" H 9350 3950 50  0001 C CNN
F 1 "VSS" H 9367 4273 50  0000 C CNN
F 2 "" H 9350 4100 50  0001 C CNN
F 3 "" H 9350 4100 50  0001 C CNN
	1    9350 4100
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:VSS-power #PWR0104
U 1 1 5E8FB3AD
P 5850 5450
F 0 "#PWR0104" H 5850 5300 50  0001 C CNN
F 1 "VSS" H 5868 5623 50  0000 C CNN
F 2 "" H 5850 5450 50  0001 C CNN
F 3 "" H 5850 5450 50  0001 C CNN
	1    5850 5450
	-1   0    0    1   
$EndComp
$Comp
L Conductivity_Sensor-rescue:VSS-power #PWR0105
U 1 1 5E8FB7E4
P 2350 2150
F 0 "#PWR0105" H 2350 2000 50  0001 C CNN
F 1 "VSS" H 2368 2323 50  0000 C CNN
F 2 "" H 2350 2150 50  0001 C CNN
F 3 "" H 2350 2150 50  0001 C CNN
	1    2350 2150
	-1   0    0    1   
$EndComp
$Comp
L Conductivity_Sensor-rescue:R-Device R3
U 1 1 5E8FC4F2
P 4750 2900
F 0 "R3" H 4820 2946 50  0000 L CNN
F 1 "1.02k" H 4820 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4680 2900 50  0001 C CNN
F 3 "~" H 4750 2900 50  0001 C CNN
F 4 "ERA-6AEB1021V" H 4750 2900 50  0001 C CNN "DigikeyPN"
	1    4750 2900
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:R-Device R1
U 1 1 5E8FCCD1
P 4400 2050
F 0 "R1" H 4470 2096 50  0000 L CNN
F 1 "1M" H 4470 2005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4330 2050 50  0001 C CNN
F 3 "~" H 4400 2050 50  0001 C CNN
F 4 "ERA-6AEB105V" H 4400 2050 50  0001 C CNN "DigikeyPN"
	1    4400 2050
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:R-Device R2
U 1 1 5E8FD13A
P 4500 3600
F 0 "R2" H 4570 3646 50  0000 L CNN
F 1 "1M" H 4570 3555 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4430 3600 50  0001 C CNN
F 3 "~" H 4500 3600 50  0001 C CNN
F 4 "ERA-6AEB105V" H 4500 3600 50  0001 C CNN "DigikeyPN"
	1    4500 3600
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:R-Device R4
U 1 1 5E8FD583
P 6000 6000
F 0 "R4" V 6100 6000 50  0000 C CNN
F 1 "8.2k" V 6200 6000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5930 6000 50  0001 C CNN
F 3 "~" H 6000 6000 50  0001 C CNN
F 4 "ERA-6AEB822V" V 6000 6000 50  0001 C CNN "DigikeyPN"
	1    6000 6000
	0    1    1    0   
$EndComp
$Comp
L Conductivity_Sensor-rescue:C-Device C6
U 1 1 5E900166
P 8800 5350
F 0 "C6" H 8915 5396 50  0000 L CNN
F 1 "47pF" H 8915 5305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8838 5200 50  0001 C CNN
F 3 "~" H 8800 5350 50  0001 C CNN
F 4 "C0805C470J5GACTU" H 8800 5350 50  0001 C CNN "DigikeyPN"
	1    8800 5350
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:D-Device D1
U 1 1 5E900B97
P 7850 4000
F 0 "D1" H 7850 3784 50  0000 C CNN
F 1 "1N4148" H 7850 3875 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 7850 4000 50  0001 C CNN
F 3 "~" H 7850 4000 50  0001 C CNN
F 4 "1N4148WT-7" H 7850 4000 50  0001 C CNN "DigikeyPN"
	1    7850 4000
	-1   0    0    1   
$EndComp
$Comp
L Conductivity_Sensor-rescue:D-Device D2
U 1 1 5E90159C
P 8450 4950
F 0 "D2" H 8450 4734 50  0000 C CNN
F 1 "1N4148" H 8450 4825 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 8450 4950 50  0001 C CNN
F 3 "~" H 8450 4950 50  0001 C CNN
F 4 "1N4148WT-7" H 8450 4950 50  0001 C CNN "DigikeyPN"
	1    8450 4950
	-1   0    0    1   
$EndComp
$Comp
L Conductivity_Sensor-rescue:VCC-power #PWR0106
U 1 1 5E9019BC
P 2350 1000
F 0 "#PWR0106" H 2350 850 50  0001 C CNN
F 1 "VCC" H 2367 1173 50  0000 C CNN
F 2 "" H 2350 1000 50  0001 C CNN
F 3 "" H 2350 1000 50  0001 C CNN
	1    2350 1000
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:VCC-power #PWR0107
U 1 1 5E9021B0
P 3550 2050
F 0 "#PWR0107" H 3550 1900 50  0001 C CNN
F 1 "VCC" H 3567 2223 50  0000 C CNN
F 2 "" H 3550 2050 50  0001 C CNN
F 3 "" H 3550 2050 50  0001 C CNN
	1    3550 2050
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:VCC-power #PWR0108
U 1 1 5E90267C
P 5850 4600
F 0 "#PWR0108" H 5850 4450 50  0001 C CNN
F 1 "VCC" H 5867 4773 50  0000 C CNN
F 2 "" H 5850 4600 50  0001 C CNN
F 3 "" H 5850 4600 50  0001 C CNN
	1    5850 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 2700 4750 2750
$Comp
L Conductivity_Sensor-rescue:VSS-power #PWR0109
U 1 1 5E90310B
P 3550 3750
F 0 "#PWR0109" H 3550 3600 50  0001 C CNN
F 1 "VSS-power" H 3568 3923 50  0000 C CNN
F 2 "" H 3550 3750 50  0001 C CNN
F 3 "" H 3550 3750 50  0001 C CNN
	1    3550 3750
	-1   0    0    1   
$EndComp
$Comp
L Conductivity_Sensor-rescue:GND-power #PWR0110
U 1 1 5E9035EE
P 5250 4900
F 0 "#PWR0110" H 5250 4650 50  0001 C CNN
F 1 "GND" H 5255 4727 50  0000 C CNN
F 2 "" H 5250 4900 50  0001 C CNN
F 3 "" H 5250 4900 50  0001 C CNN
	1    5250 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 1800 2350 1900
Wire Wire Line
	2150 1600 2050 1600
Wire Wire Line
	2050 1600 2050 2900
Wire Wire Line
	2050 2900 2750 2900
Wire Wire Line
	3550 2300 3550 2250
Wire Wire Line
	2350 1000 2350 1050
Wire Wire Line
	4500 3300 4500 3450
Wire Wire Line
	4150 3100 4750 3100
Wire Wire Line
	4750 3050 4750 3100
Wire Wire Line
	4750 2700 4150 2700
Wire Wire Line
	4500 3300 4150 3300
Wire Wire Line
	4400 2500 4150 2500
Wire Wire Line
	4400 2200 4400 2500
Wire Wire Line
	4400 1900 4400 1500
Wire Wire Line
	4400 1500 2750 1500
Wire Wire Line
	8600 4950 8800 4950
Wire Wire Line
	8800 4950 8800 5200
Connection ~ 8800 4950
Wire Wire Line
	9150 4950 8800 4950
$Comp
L Conductivity_Sensor-rescue:GND-power #PWR0112
U 1 1 5E917328
P 8800 5700
F 0 "#PWR0112" H 8800 5450 50  0001 C CNN
F 1 "GND" H 8805 5527 50  0000 C CNN
F 2 "" H 8800 5700 50  0001 C CNN
F 3 "" H 8800 5700 50  0001 C CNN
	1    8800 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 5500 8800 5700
$Comp
L Conductivity_Sensor-rescue:VCC-power #PWR0113
U 1 1 5E917DD6
P 9350 5450
F 0 "#PWR0113" H 9350 5300 50  0001 C CNN
F 1 "VCC" H 9368 5623 50  0000 C CNN
F 2 "" H 9350 5450 50  0001 C CNN
F 3 "" H 9350 5450 50  0001 C CNN
	1    9350 5450
	-1   0    0    1   
$EndComp
Wire Wire Line
	9350 4100 9350 4150
Wire Wire Line
	8000 4000 8150 4000
Wire Wire Line
	8150 4000 8150 4950
Wire Wire Line
	8150 4950 8300 4950
Wire Wire Line
	9150 4750 8950 4750
Wire Wire Line
	8950 4750 8950 3700
Wire Wire Line
	8950 3700 8250 3700
Wire Wire Line
	7900 4950 8150 4950
Connection ~ 8150 4950
Wire Wire Line
	7300 4850 7200 4850
Wire Wire Line
	7200 4850 7200 4000
Wire Wire Line
	7200 4000 7700 4000
Wire Wire Line
	7950 3700 7200 3700
Wire Wire Line
	7200 3700 7200 4000
Connection ~ 7200 4000
Wire Wire Line
	7500 4650 7500 4250
$Comp
L Conductivity_Sensor-rescue:VCC-power #PWR0114
U 1 1 5E920E1A
P 7500 5400
F 0 "#PWR0114" H 7500 5250 50  0001 C CNN
F 1 "VCC" H 7518 5573 50  0000 C CNN
F 2 "" H 7500 5400 50  0001 C CNN
F 3 "" H 7500 5400 50  0001 C CNN
	1    7500 5400
	-1   0    0    1   
$EndComp
Wire Wire Line
	7500 5250 7500 5350
Wire Wire Line
	6250 5050 6450 5050
Wire Wire Line
	6150 6000 6450 6000
Wire Wire Line
	6450 6000 6450 5050
Connection ~ 6450 5050
Wire Wire Line
	6450 5050 7300 5050
Wire Wire Line
	5850 6000 5500 6000
Wire Wire Line
	5500 6000 5500 5150
Wire Wire Line
	5500 5150 5650 5150
Wire Wire Line
	5650 4950 5550 4950
Wire Wire Line
	5550 4950 5550 4800
Wire Wire Line
	5550 4800 5250 4800
Wire Wire Line
	5250 4800 5250 4900
Wire Wire Line
	5850 4600 5850 4700
Wire Wire Line
	4500 3300 5300 3300
Connection ~ 4500 3300
Wire Wire Line
	4400 2500 5300 2500
Connection ~ 4400 2500
Wire Wire Line
	4400 1500 5150 1500
Wire Wire Line
	5150 1500 5150 2350
Connection ~ 4400 1500
Connection ~ 5150 1500
Wire Wire Line
	5450 2850 5800 2850
Wire Wire Line
	5450 2950 5800 2950
Text Label 1000 1100 0    50   ~ 0
Vexc
Text Label 5150 1300 0    50   ~ 0
Vouter
Wire Wire Line
	9750 4850 10050 4850
Text Label 10100 4850 0    50   ~ 0
Voutput
Text Notes 7700 3400 0    50   ~ 0
Precision Peak Detector\n\n
Text Notes 5200 6450 0    50   ~ 0
For R=8.2k and a cell constant of 1.3/cm,\nthe output voltage would up to 5V for\nthe desired range (2-34mS/cm ==> 0.3-4.9V)
Text Notes 10100 4650 0    50   ~ 0
output from 0.3V to 4.9V\n(2mS/cm − 34mS/cm)
Text Notes 650  2500 0    50   ~ 0
Excitation voltage.\n- Square signal.\n- 0 DC bias\n- 1-10 kHz\n- Vpeak = 1V
Text Notes 5500 1550 0    50   ~ 0
Voltage applied on the sensor\nThis signal can be used to\ncheck is sensor is defect or not\npresent.
Text Notes 6350 2500 0    50   ~ 0
Voltage on the inner\nelectrodes of the\nconductivity sensor\nis Vexc/50 = 20 mV (peak)
Text Notes 2500 4200 0    50   ~ 0
Gain is set to 50.\nThus,\nV (inner electrodes) = Vexc/50
$Comp
L Conductivity_Sensor-rescue:Conn_01x06-Connector_Generic LFS1107
U 1 1 5E93A510
P 6000 2850
F 0 "LFS1107" H 6080 2842 50  0000 L CNN
F 1 "Conn_01x06" H 6080 2751 50  0000 L CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_1x06_P1.27mm_Vertical" H 6000 2850 50  0001 C CNN
F 3 "~" H 6000 2850 50  0001 C CNN
F 4 "SMS-106-02-G-S" H 6000 2850 50  0001 C CNN "DigikeyPN"
	1    6000 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 2350 5400 2350
Wire Wire Line
	5300 3300 5300 3050
Wire Wire Line
	5300 3050 5800 3050
Wire Wire Line
	5400 3450 5400 3150
Wire Wire Line
	5400 3150 5800 3150
Wire Wire Line
	5000 3450 5400 3450
Text Label 5450 2850 0    50   ~ 0
RTD_1
Text Label 5450 2950 0    50   ~ 0
RTD_2
$Comp
L Conductivity_Sensor-rescue:Conn_01x01_Female-Connector Vexc1
U 1 1 5E962651
P 1250 750
F 0 "Vexc1" H 1278 776 50  0000 L CNN
F 1 "Conn_01x01_Female" H 1278 685 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1.2mm" H 1250 750 50  0001 C CNN
F 3 "~" H 1250 750 50  0001 C CNN
	1    1250 750 
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:C-Device C2
U 1 1 5E9635F7
P 3150 2250
F 0 "C2" V 2898 2250 50  0000 C CNN
F 1 "0.1uF" V 2989 2250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3188 2100 50  0001 C CNN
F 3 "~" H 3150 2250 50  0001 C CNN
F 4 "C0805C104M5RACTU" V 3150 2250 50  0001 C CNN "DigikeyPN"
	1    3150 2250
	0    1    1    0   
$EndComp
$Comp
L Conductivity_Sensor-rescue:C-Device C1
U 1 1 5E963EC5
P 2800 1050
F 0 "C1" V 2600 1150 50  0000 C CNN
F 1 "0.1uF" V 2700 1200 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2838 900 50  0001 C CNN
F 3 "~" H 2800 1050 50  0001 C CNN
F 4 "C0805C104M5RACTU" V 2800 1050 50  0001 C CNN "DigikeyPN"
	1    2800 1050
	0    1    1    0   
$EndComp
$Comp
L Conductivity_Sensor-rescue:GND-power #PWR0115
U 1 1 5E965800
P 2900 2400
F 0 "#PWR0115" H 2900 2150 50  0001 C CNN
F 1 "GND" H 2905 2227 50  0000 C CNN
F 2 "" H 2900 2400 50  0001 C CNN
F 3 "" H 2900 2400 50  0001 C CNN
	1    2900 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 2250 2900 2250
Wire Wire Line
	3300 2250 3550 2250
Connection ~ 3550 2250
Wire Wire Line
	3550 2250 3550 2050
Wire Wire Line
	2650 1050 2450 1050
Connection ~ 2350 1050
Wire Wire Line
	2350 1050 2350 1200
$Comp
L Conductivity_Sensor-rescue:GND-power #PWR0116
U 1 1 5E96D721
P 3200 1150
F 0 "#PWR0116" H 3200 900 50  0001 C CNN
F 1 "GND" H 3205 977 50  0000 C CNN
F 2 "" H 3200 1150 50  0001 C CNN
F 3 "" H 3200 1150 50  0001 C CNN
	1    3200 1150
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:C-Device C3
U 1 1 5E970450
P 6150 4700
F 0 "C3" V 5898 4700 50  0000 C CNN
F 1 "0.1uF" V 5989 4700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6188 4550 50  0001 C CNN
F 3 "~" H 6150 4700 50  0001 C CNN
F 4 "C0805C104M5RACTU" V 6150 4700 50  0001 C CNN "DigikeyPN"
	1    6150 4700
	0    1    1    0   
$EndComp
$Comp
L Conductivity_Sensor-rescue:C-Device C4
U 1 1 5E970BDB
P 7200 5350
F 0 "C4" V 6948 5350 50  0000 C CNN
F 1 "0.1uF" V 7039 5350 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7238 5200 50  0001 C CNN
F 3 "~" H 7200 5350 50  0001 C CNN
F 4 "C0805C104M5RACTU" V 7200 5350 50  0001 C CNN "DigikeyPN"
	1    7200 5350
	0    1    1    0   
$EndComp
$Comp
L Conductivity_Sensor-rescue:C-Device C5
U 1 1 5E971168
P 9750 5300
F 0 "C5" V 9498 5300 50  0000 C CNN
F 1 "0.1uF" V 9589 5300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9788 5150 50  0001 C CNN
F 3 "~" H 9750 5300 50  0001 C CNN
F 4 "C0805C104M5RACTU" V 9750 5300 50  0001 C CNN "DigikeyPN"
	1    9750 5300
	0    1    1    0   
$EndComp
Wire Wire Line
	9600 5300 9350 5300
Wire Wire Line
	9350 5300 9350 5450
$Comp
L Conductivity_Sensor-rescue:GND-power #PWR0117
U 1 1 5E9769DD
P 10100 5450
F 0 "#PWR0117" H 10100 5200 50  0001 C CNN
F 1 "GND" H 10105 5277 50  0000 C CNN
F 2 "" H 10100 5450 50  0001 C CNN
F 3 "" H 10100 5450 50  0001 C CNN
	1    10100 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 5300 10100 5300
Wire Wire Line
	10100 5300 10100 5450
Wire Wire Line
	7350 5350 7500 5350
Connection ~ 7500 5350
Wire Wire Line
	7500 5350 7500 5400
$Comp
L Conductivity_Sensor-rescue:GND-power #PWR0118
U 1 1 5E97B356
P 6950 5500
F 0 "#PWR0118" H 6950 5250 50  0001 C CNN
F 1 "GND" H 6955 5327 50  0000 C CNN
F 2 "" H 6950 5500 50  0001 C CNN
F 3 "" H 6950 5500 50  0001 C CNN
	1    6950 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 5500 6950 5350
Wire Wire Line
	6950 5350 7050 5350
Wire Wire Line
	1050 750  1000 750 
Wire Wire Line
	1000 750  1000 800 
$Comp
L Conductivity_Sensor-rescue:Conn_01x01_Female-Connector Vouter1
U 1 1 5E980660
P 5350 1000
F 0 "Vouter1" H 5378 1026 50  0000 L CNN
F 1 "Conn_01x01_Female" H 5378 935 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1.2mm" H 5350 1000 50  0001 C CNN
F 3 "~" H 5350 1000 50  0001 C CNN
	1    5350 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 4700 5850 4700
Connection ~ 5850 4700
Wire Wire Line
	5850 4700 5850 4750
$Comp
L Conductivity_Sensor-rescue:GND-power #PWR0119
U 1 1 5E985C5B
P 6450 4800
F 0 "#PWR0119" H 6450 4550 50  0001 C CNN
F 1 "GND" H 6455 4627 50  0000 C CNN
F 2 "" H 6450 4800 50  0001 C CNN
F 3 "" H 6450 4800 50  0001 C CNN
	1    6450 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 4800 6450 4700
Wire Wire Line
	6450 4700 6300 4700
$Comp
L Conductivity_Sensor-rescue:R-Device R5
U 1 1 5E8FFC07
P 8100 3700
F 0 "R5" V 7893 3700 50  0000 C CNN
F 1 "1k" V 7984 3700 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8030 3700 50  0001 C CNN
F 3 "~" H 8100 3700 50  0001 C CNN
F 4 "ERA-6AEB102V" V 8100 3700 50  0001 C CNN "DigikeyPN"
	1    8100 3700
	0    1    1    0   
$EndComp
$Comp
L Conductivity_Sensor-rescue:Conn_01x01_Female-Connector Voutput1
U 1 1 5E98EA9F
P 10550 4850
F 0 "Voutput1" H 10350 4800 50  0000 L CNN
F 1 "Conn_01x01_Female" H 10100 4700 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1.2mm" H 10550 4850 50  0001 C CNN
F 3 "~" H 10550 4850 50  0001 C CNN
	1    10550 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 5150 9350 5300
Connection ~ 9350 5300
Wire Wire Line
	3550 3500 3550 3600
NoConn ~ 5450 2850
NoConn ~ 5450 2950
Wire Wire Line
	5000 5150 5500 5150
Wire Wire Line
	5000 3450 5000 5150
Connection ~ 5500 5150
$Comp
L Conductivity_Sensor-rescue:PWR_FLAG-power #FLG0101
U 1 1 5E9E5BCC
P 2450 1050
F 0 "#FLG0101" H 2450 1125 50  0001 C CNN
F 1 "PWR_FLAG" H 2550 1200 50  0000 C CNN
F 2 "" H 2450 1050 50  0001 C CNN
F 3 "~" H 2450 1050 50  0001 C CNN
	1    2450 1050
	1    0    0    -1  
$EndComp
Connection ~ 2450 1050
Wire Wire Line
	2450 1050 2350 1050
$Comp
L Conductivity_Sensor-rescue:PWR_FLAG-power #FLG0102
U 1 1 5E9E7540
P 2200 1900
F 0 "#FLG0102" H 2200 1975 50  0001 C CNN
F 1 "PWR_FLAG" H 2300 1850 50  0000 C CNN
F 2 "" H 2200 1900 50  0001 C CNN
F 3 "~" H 2200 1900 50  0001 C CNN
	1    2200 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 1900 2350 1900
Connection ~ 2350 1900
Wire Wire Line
	2350 1900 2350 2150
$Comp
L Conductivity_Sensor-rescue:PWR_FLAG-power #FLG0104
U 1 1 5E9EDCA9
P 800 800
F 0 "#FLG0104" H 800 875 50  0001 C CNN
F 1 "PWR_FLAG" H 900 950 50  0000 C CNN
F 2 "" H 800 800 50  0001 C CNN
F 3 "~" H 800 800 50  0001 C CNN
	1    800  800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  800  1000 800 
Connection ~ 1000 800 
Wire Wire Line
	1000 800  1000 1400
$Comp
L Conductivity_Sensor-rescue:PWR_FLAG-power #FLG0105
U 1 1 5E9F1521
P 3200 950
F 0 "#FLG0105" H 3200 1025 50  0001 C CNN
F 1 "PWR_FLAG" H 3300 1100 50  0000 C CNN
F 2 "" H 3200 950 50  0001 C CNN
F 3 "~" H 3200 950 50  0001 C CNN
	1    3200 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 1000 5150 1500
Wire Wire Line
	2950 1050 3200 1050
Wire Wire Line
	3200 950  3200 1050
Wire Wire Line
	3200 1150 3200 1050
Connection ~ 3200 1050
Wire Wire Line
	5400 2750 5800 2750
Wire Wire Line
	5400 2350 5400 2750
Wire Wire Line
	5300 2500 5300 2650
Wire Wire Line
	5300 2650 5800 2650
$Comp
L Conductivity_Sensor-rescue:GND-power #PWR0111
U 1 1 5E907253
P 4500 4100
F 0 "#PWR0111" H 4500 3850 50  0001 C CNN
F 1 "GND" H 4505 3927 50  0000 C CNN
F 2 "" H 4500 4100 50  0001 C CNN
F 3 "" H 4500 4100 50  0001 C CNN
	1    4500 4100
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:C-Device C?
U 1 1 6011F49D
P 9700 4300
F 0 "C?" V 9448 4300 50  0000 C CNN
F 1 "0.1uF" V 9539 4300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9738 4150 50  0001 C CNN
F 3 "~" H 9700 4300 50  0001 C CNN
F 4 "C0805C104M5RACTU" V 9700 4300 50  0001 C CNN "DigikeyPN"
	1    9700 4300
	-1   0    0    1   
$EndComp
Wire Wire Line
	9700 4150 9350 4150
Connection ~ 9350 4150
Wire Wire Line
	9350 4150 9350 4550
$Comp
L Conductivity_Sensor-rescue:GND-power #PWR?
U 1 1 60126C1B
P 9700 4500
F 0 "#PWR?" H 9700 4250 50  0001 C CNN
F 1 "GND" H 9705 4327 50  0000 C CNN
F 2 "" H 9700 4500 50  0001 C CNN
F 3 "" H 9700 4500 50  0001 C CNN
	1    9700 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 4500 9700 4450
Wire Wire Line
	8950 3700 10050 3700
Wire Wire Line
	10050 3700 10050 4850
Connection ~ 8950 3700
Connection ~ 10050 4850
Wire Wire Line
	10050 4850 10350 4850
$Comp
L Conductivity_Sensor-rescue:C-Device C?
U 1 1 6015CBF5
P 7850 4400
F 0 "C?" V 7598 4400 50  0000 C CNN
F 1 "0.1uF" V 7689 4400 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7888 4250 50  0001 C CNN
F 3 "~" H 7850 4400 50  0001 C CNN
F 4 "C0805C104M5RACTU" V 7850 4400 50  0001 C CNN "DigikeyPN"
	1    7850 4400
	-1   0    0    1   
$EndComp
$Comp
L Conductivity_Sensor-rescue:C-Device C?
U 1 1 6015D06C
P 6050 5500
F 0 "C?" H 5798 5500 50  0000 C CNN
F 1 "0.1uF" H 5850 5550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6088 5350 50  0001 C CNN
F 3 "~" H 6050 5500 50  0001 C CNN
F 4 "C0805C104M5RACTU" V 6050 5500 50  0001 C CNN "DigikeyPN"
	1    6050 5500
	-1   0    0    1   
$EndComp
Wire Wire Line
	6050 5350 5850 5350
Connection ~ 5850 5350
Wire Wire Line
	5850 5350 5850 5450
$Comp
L Conductivity_Sensor-rescue:GND-power #PWR?
U 1 1 6016E832
P 6050 5700
F 0 "#PWR?" H 6050 5450 50  0001 C CNN
F 1 "GND" H 6055 5527 50  0000 C CNN
F 2 "" H 6050 5700 50  0001 C CNN
F 3 "" H 6050 5700 50  0001 C CNN
	1    6050 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 5650 6050 5700
Wire Wire Line
	7850 4250 7500 4250
Connection ~ 7500 4250
Wire Wire Line
	7500 4250 7500 4200
$Comp
L Conductivity_Sensor-rescue:GND-power #PWR?
U 1 1 601A2D89
P 7850 4600
F 0 "#PWR?" H 7850 4350 50  0001 C CNN
F 1 "GND" H 7855 4427 50  0000 C CNN
F 2 "" H 7850 4600 50  0001 C CNN
F 3 "" H 7850 4600 50  0001 C CNN
	1    7850 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 4600 7850 4550
$Comp
L Conductivity_Sensor-rescue:C-Device C?
U 1 1 601BD341
P 2600 2050
F 0 "C?" V 2348 2050 50  0000 C CNN
F 1 "0.1uF" V 2439 2050 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2638 1900 50  0001 C CNN
F 3 "~" H 2600 2050 50  0001 C CNN
F 4 "C0805C104M5RACTU" V 2600 2050 50  0001 C CNN "DigikeyPN"
	1    2600 2050
	-1   0    0    1   
$EndComp
Wire Wire Line
	2350 1900 2600 1900
Wire Wire Line
	2900 2250 2900 2400
Wire Wire Line
	2600 2200 2600 2250
Wire Wire Line
	2600 2250 2900 2250
Connection ~ 2900 2250
$Comp
L Conductivity_Sensor-rescue:R-Device R?
U 1 1 601D6A1B
P 1000 1650
F 0 "R?" H 1070 1696 50  0000 L CNN
F 1 "470" H 1070 1605 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 930 1650 50  0001 C CNN
F 3 "~" H 1000 1650 50  0001 C CNN
F 4 "ERA-6AEB1021V" H 1000 1650 50  0001 C CNN "DigikeyPN"
	1    1000 1650
	1    0    0    -1  
$EndComp
$Comp
L Conductivity_Sensor-rescue:C-Device C?
U 1 1 601DE309
P 1500 1400
F 0 "C?" V 1300 1500 50  0000 C CNN
F 1 "0.1uF" V 1400 1550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1538 1250 50  0001 C CNN
F 3 "~" H 1500 1400 50  0001 C CNN
F 4 "C0805C104M5RACTU" V 1500 1400 50  0001 C CNN "DigikeyPN"
	1    1500 1400
	0    1    1    0   
$EndComp
Wire Wire Line
	1350 1400 1000 1400
Wire Wire Line
	1000 1500 1000 1400
Connection ~ 1000 1400
Wire Wire Line
	1650 1400 2150 1400
$Comp
L Conductivity_Sensor-rescue:GND-power #PWR?
U 1 1 601EA61E
P 1000 1900
F 0 "#PWR?" H 1000 1650 50  0001 C CNN
F 1 "GND" H 1005 1727 50  0000 C CNN
F 2 "" H 1000 1900 50  0001 C CNN
F 3 "" H 1000 1900 50  0001 C CNN
	1    1000 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 1800 1000 1900
$Comp
L Conductivity_Sensor-rescue:C-Device C?
U 1 1 601EEC62
P 3900 3800
F 0 "C?" H 3850 4100 50  0000 C CNN
F 1 "0.1uF" H 3900 4000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3938 3650 50  0001 C CNN
F 3 "~" H 3900 3800 50  0001 C CNN
F 4 "C0805C104M5RACTU" V 3900 3800 50  0001 C CNN "DigikeyPN"
	1    3900 3800
	-1   0    0    1   
$EndComp
Wire Wire Line
	3900 3600 3550 3600
Connection ~ 3550 3600
Wire Wire Line
	3550 3600 3550 3750
Wire Wire Line
	4500 3750 4500 3950
Wire Wire Line
	3900 3950 4500 3950
Connection ~ 4500 3950
Wire Wire Line
	4500 3950 4500 4100
Wire Wire Line
	3900 3600 3900 3650
$EndSCHEMATC
