EESchema Schematic File Version 4
LIBS:subbat-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 9 14
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
L device:Q_PMOS_GSD Q2
U 1 1 5A6914EB
P 2000 1300
F 0 "Q2" V 2343 1300 50  0000 C CNN
F 1 "Q_PMOS_GSD" V 2252 1300 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 2200 1400 50  0001 C CNN
F 3 "" H 2000 1300 50  0001 C CNN
	1    2000 1300
	0    1    -1   0   
$EndComp
$Comp
L device:R R2
U 1 1 5A6914F2
P 1650 1400
F 0 "R2" H 1720 1446 50  0000 L CNN
F 1 "470k" H 1720 1355 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1580 1400 50  0001 C CNN
F 3 "" H 1650 1400 50  0001 C CNN
	1    1650 1400
	1    0    0    -1  
$EndComp
$Comp
L device:Q_NPN_BEC Q1
U 1 1 5A6914F9
P 1900 1850
F 0 "Q1" H 2091 1896 50  0000 L CNN
F 1 "Q_NPN_BEC" H 2091 1805 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 2100 1950 50  0001 C CNN
F 3 "" H 1900 1850 50  0001 C CNN
	1    1900 1850
	1    0    0    -1  
$EndComp
$Comp
L device:R R1
U 1 1 5A691500
P 1600 2000
F 0 "R1" H 1670 2046 50  0000 L CNN
F 1 "10k" H 1670 1955 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1530 2000 50  0001 C CNN
F 3 "" H 1600 2000 50  0001 C CNN
	1    1600 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 1850 1700 1850
Wire Wire Line
	2000 2150 2000 2050
Wire Wire Line
	1050 1200 1650 1200
Wire Wire Line
	1650 1250 1650 1200
Connection ~ 1650 1200
Wire Wire Line
	1650 1200 1800 1200
Wire Wire Line
	1650 1550 2000 1550
Wire Wire Line
	2000 1550 2000 1500
Wire Wire Line
	2000 1650 2000 1550
Connection ~ 2000 1550
$Comp
L power:GND #PWR017
U 1 1 5A691511
P 2000 2150
F 0 "#PWR017" H 2000 1900 50  0001 C CNN
F 1 "GND" H 2005 1977 50  0000 C CNN
F 2 "" H 2000 2150 50  0001 C CNN
F 3 "" H 2000 2150 50  0001 C CNN
	1    2000 2150
	1    0    0    -1  
$EndComp
$Comp
L device:D_Schottky D4
U 1 1 5A691519
P 2950 1200
F 0 "D4" H 2950 1416 50  0000 C CNN
F 1 "D_Schottky" H 2950 1325 50  0000 C CNN
F 2 "Diodes_SMD:D_SMA" H 2950 1200 50  0001 C CNN
F 3 "" H 2950 1200 50  0001 C CNN
	1    2950 1200
	-1   0    0    -1  
$EndComp
$Comp
L device:D_Schottky D5
U 1 1 5A691521
P 2950 1550
F 0 "D5" H 2950 1766 50  0000 C CNN
F 1 "D_Schottky" H 2950 1675 50  0000 C CNN
F 2 "Diodes_SMD:D_SMA" H 2950 1550 50  0001 C CNN
F 3 "" H 2950 1550 50  0001 C CNN
	1    2950 1550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2200 1200 2450 1200
Wire Wire Line
	2350 1550 2800 1550
Wire Wire Line
	3100 1550 3250 1550
Wire Wire Line
	3250 1550 3250 1200
Wire Wire Line
	3250 1200 3100 1200
Wire Wire Line
	5000 1200 5000 1150
$Comp
L device:R R3
U 1 1 5A691575
P 5150 1350
F 0 "R3" H 5220 1396 50  0000 L CNN
F 1 "2.2k" H 5220 1305 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5080 1350 50  0001 C CNN
F 3 "" H 5150 1350 50  0001 C CNN
	1    5150 1350
	1    0    0    -1  
$EndComp
$Comp
L device:LED D6
U 1 1 5A69157C
P 5150 1700
F 0 "D6" V 5188 1583 50  0000 R CNN
F 1 "LED" V 5097 1583 50  0000 R CNN
F 2 "LEDs:LED_D3.0mm" H 5150 1700 50  0001 C CNN
F 3 "~" H 5150 1700 50  0001 C CNN
	1    5150 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5A691583
P 5150 1900
F 0 "#PWR021" H 5150 1650 50  0001 C CNN
F 1 "GND" H 5155 1727 50  0000 C CNN
F 2 "" H 5150 1900 50  0001 C CNN
F 3 "" H 5150 1900 50  0001 C CNN
	1    5150 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 1550 5150 1500
Wire Wire Line
	5150 1900 5150 1850
Wire Wire Line
	5000 1200 5150 1200
Text HLabel 1050 1200 0    50   Input ~ 0
VA
Text HLabel 2350 1550 0    50   Input ~ 0
VS
Text HLabel 1600 2150 3    50   Input ~ 0
~DEEP_SLEEP_EN
Text HLabel 2300 850  0    50   Input ~ 0
VA_G
Wire Wire Line
	2300 850  2450 850 
Wire Wire Line
	2450 850  2450 1200
Connection ~ 2450 1200
Wire Wire Line
	2450 1200 2800 1200
$Comp
L power:+5V #PWR077
U 1 1 5A6F5DF5
P 5000 1150
F 0 "#PWR077" H 5000 1000 50  0001 C CNN
F 1 "+5V" H 5015 1323 50  0000 C CNN
F 2 "" H 5000 1150 50  0001 C CNN
F 3 "" H 5000 1150 50  0001 C CNN
	1    5000 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 1500 4800 1700
$Comp
L power:GND #PWR074
U 1 1 59E4DB95
P 4800 1700
F 0 "#PWR074" H 4800 1450 50  0001 C CNN
F 1 "GND" H 4800 1550 50  0000 C CNN
F 2 "" H 4800 1700 50  0000 C CNN
F 3 "" H 4800 1700 50  0000 C CNN
	1    4800 1700
	-1   0    0    -1  
$EndComp
$Comp
L device:C C69
U 1 1 59E4DB9B
P 4800 1350
F 0 "C69" H 4750 750 50  0000 L CNN
F 1 "10u" H 4750 650 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4915 1259 50  0001 L CNN
F 3 "" H 4800 1350 50  0000 C CNN
	1    4800 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 1550 3600 1500
$Comp
L power:GND #PWR068
U 1 1 59E4DBA8
P 3600 1550
F 0 "#PWR068" H 3600 1300 50  0001 C CNN
F 1 "GND" H 3600 1400 50  0000 C CNN
F 2 "" H 3600 1550 50  0000 C CNN
F 3 "" H 3600 1550 50  0000 C CNN
	1    3600 1550
	-1   0    0    -1  
$EndComp
$Comp
L device:C C4
U 1 1 59E4DBAE
P 3600 1350
F 0 "C4" H 3550 900 50  0000 L CNN
F 1 "4.7u 50V" H 3550 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3715 1259 50  0001 L CNN
F 3 "" H 3600 1350 50  0000 C CNN
	1    3600 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 1200 4800 1200
Connection ~ 3600 1200
Wire Wire Line
	3600 1200 3950 1200
Wire Wire Line
	3250 1200 3600 1200
Connection ~ 3250 1200
$Comp
L regul:L7805 U2
U 1 1 5A7C15A3
P 4250 1200
F 0 "U2" H 4250 1442 50  0000 C CNN
F 1 "L7803" H 4250 1351 50  0000 C CNN
F 2 "TO_SOT_Packages_THT:TO-220-3_Vertical" H 4275 1050 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 4250 1150 50  0001 C CNN
	1    4250 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR071
U 1 1 5A7C15E1
P 4250 1500
F 0 "#PWR071" H 4250 1250 50  0001 C CNN
F 1 "GND" H 4250 1350 50  0000 C CNN
F 2 "" H 4250 1500 50  0000 C CNN
F 3 "" H 4250 1500 50  0000 C CNN
	1    4250 1500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4800 1200 5000 1200
Connection ~ 4800 1200
Connection ~ 5000 1200
Text Label 3700 1200 0    50   ~ 0
REG_IN
$Comp
L DeeComponents:TPS54202 U11
U 1 1 5A71FFA9
P 4200 2850
F 0 "U11" H 4200 3317 50  0000 C CNN
F 1 "TPS54202" H 4200 3226 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-6" H 4200 2350 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/tps54202.pdf" H 3800 3300 50  0001 C CNN
	1    4200 2850
	1    0    0    -1  
$EndComp
Text Label 3050 2650 0    50   ~ 0
REG_IN
NoConn ~ 3700 2850
$Comp
L power:GND #PWR044
U 1 1 5A72087F
P 4200 3250
F 0 "#PWR044" H 4200 3000 50  0001 C CNN
F 1 "GND" H 4200 3100 50  0000 C CNN
F 2 "" H 4200 3250 50  0000 C CNN
F 3 "" H 4200 3250 50  0000 C CNN
	1    4200 3250
	-1   0    0    -1  
$EndComp
$Comp
L device:C C29
U 1 1 5A72109C
P 5000 2650
F 0 "C29" V 5200 2750 50  0000 L CNN
F 1 "0.1u 50V" V 5150 2650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5115 2559 50  0001 L CNN
F 3 "" H 5000 2650 50  0000 C CNN
	1    5000 2650
	0    -1   -1   0   
$EndComp
$Comp
L device:L_Small L1
U 1 1 5A721DA3
P 5350 2850
F 0 "L1" V 5172 2850 50  0000 C CNN
F 1 "22u" V 5263 2850 50  0000 C CNN
F 2 "components:Choke_SMD_7.3x7.3_H3.5_handsoldering" H 5350 2850 50  0001 C CNN
F 3 "" H 5350 2850 50  0001 C CNN
	1    5350 2850
	0    1    1    0   
$EndComp
$Comp
L device:C C30
U 1 1 5A722483
P 5650 3000
F 0 "C30" H 5750 2850 50  0000 L CNN
F 1 "10u" H 5750 2750 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 5765 2909 50  0001 L CNN
F 3 "" H 5650 3000 50  0000 C CNN
	1    5650 3000
	1    0    0    -1  
$EndComp
$Comp
L device:R R11
U 1 1 5A723211
P 6300 3350
F 0 "R11" H 6370 3396 50  0000 L CNN
F 1 "100k" H 6370 3305 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6230 3350 50  0001 C CNN
F 3 "" H 6300 3350 50  0001 C CNN
	1    6300 3350
	1    0    0    -1  
$EndComp
$Comp
L device:C C31
U 1 1 5A7234D1
P 6100 3350
F 0 "C31" H 5950 3450 50  0000 L CNN
F 1 "75p" H 5950 3250 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Disc_D3.0mm_W1.6mm_P2.50mm" H 6215 3259 50  0001 L CNN
F 3 "" H 6100 3350 50  0000 C CNN
	1    6100 3350
	1    0    0    -1  
$EndComp
$Comp
L device:R R63
U 1 1 5A7238A8
P 6300 4050
F 0 "R63" H 6370 4096 50  0000 L CNN
F 1 "2.2k" H 6370 4005 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6230 4050 50  0001 C CNN
F 3 "" H 6300 4050 50  0001 C CNN
	1    6300 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR047
U 1 1 5A723CA0
P 6300 4600
F 0 "#PWR047" H 6300 4350 50  0001 C CNN
F 1 "GND" H 6300 4450 50  0000 C CNN
F 2 "" H 6300 4600 50  0000 C CNN
F 3 "" H 6300 4600 50  0000 C CNN
	1    6300 4600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4700 2650 4850 2650
Wire Wire Line
	5150 2650 5150 2850
Wire Wire Line
	5150 2850 5250 2850
Wire Wire Line
	5150 2850 4700 2850
Connection ~ 5150 2850
Wire Wire Line
	5450 2850 5650 2850
Wire Wire Line
	5650 2850 6300 2850
Connection ~ 5650 2850
$Comp
L power:+5V #PWR046
U 1 1 5A725468
P 6300 2850
F 0 "#PWR046" H 6300 2700 50  0001 C CNN
F 1 "+5V" H 6315 3023 50  0000 C CNN
F 2 "" H 6300 2850 50  0001 C CNN
F 3 "" H 6300 2850 50  0001 C CNN
	1    6300 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 3200 6300 3200
Connection ~ 6300 3200
Wire Wire Line
	6300 3500 6300 3550
Wire Wire Line
	6300 3550 6100 3550
Wire Wire Line
	6100 3550 6100 3500
Connection ~ 6300 3550
Wire Wire Line
	6300 3550 6300 3600
Wire Wire Line
	6100 3550 5050 3550
Wire Wire Line
	5050 3550 5050 3050
Wire Wire Line
	5050 3050 4700 3050
Connection ~ 6100 3550
Wire Wire Line
	6300 4200 6300 4300
$Comp
L power:GND #PWR045
U 1 1 5A7277B8
P 5650 3150
F 0 "#PWR045" H 5650 2900 50  0001 C CNN
F 1 "GND" H 5650 3000 50  0000 C CNN
F 2 "" H 5650 3150 50  0000 C CNN
F 3 "" H 5650 3150 50  0000 C CNN
	1    5650 3150
	-1   0    0    -1  
$EndComp
$Comp
L device:R R62
U 1 1 5A728925
P 6300 3750
F 0 "R62" H 6370 3796 50  0000 L CNN
F 1 "10k" H 6370 3705 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6230 3750 50  0001 C CNN
F 3 "" H 6300 3750 50  0001 C CNN
	1    6300 3750
	1    0    0    -1  
$EndComp
Text Notes 6600 3900 0    50   ~ 0
Vout = 0.596*((100000.0)/(10000.0+2200.0+1000.0)+1)\n=> 5.111151515151515\n
Wire Wire Line
	6300 2850 6300 3200
Connection ~ 6300 2850
$Comp
L device:R R10
U 1 1 5A722BEC
P 6300 4450
F 0 "R10" H 6370 4496 50  0000 L CNN
F 1 "1k" H 6370 4405 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6230 4450 50  0001 C CNN
F 3 "" H 6300 4450 50  0001 C CNN
	1    6300 4450
	1    0    0    -1  
$EndComp
$Comp
L device:C C32
U 1 1 5A730079
P 3450 2850
F 0 "C32" H 3300 2950 50  0000 L CNN
F 1 "1u" H 3300 2750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3565 2759 50  0001 L CNN
F 3 "" H 3450 2850 50  0000 C CNN
	1    3450 2850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR050
U 1 1 5A730D32
P 3450 3000
F 0 "#PWR050" H 3450 2750 50  0001 C CNN
F 1 "GND" H 3450 2850 50  0000 C CNN
F 2 "" H 3450 3000 50  0000 C CNN
F 3 "" H 3450 3000 50  0000 C CNN
	1    3450 3000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3050 2650 3450 2650
Wire Wire Line
	3450 2700 3450 2650
Connection ~ 3450 2650
Wire Wire Line
	3450 2650 3700 2650
Text Label 4750 2850 0    50   ~ 0
VSW
$Comp
L device:R R67
U 1 1 5A760AB8
P 3200 4050
F 0 "R67" H 3270 4096 50  0000 L CNN
F 1 "2.2k" H 3270 4005 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 4050 50  0001 C CNN
F 3 "" H 3200 4050 50  0001 C CNN
	1    3200 4050
	1    0    0    -1  
$EndComp
$Comp
L device:LED D36
U 1 1 5A760ABE
P 3200 4400
F 0 "D36" V 3238 4283 50  0000 R CNN
F 1 "LED" V 3147 4283 50  0000 R CNN
F 2 "LEDs:LED_D3.0mm" H 3200 4400 50  0001 C CNN
F 3 "~" H 3200 4400 50  0001 C CNN
	1    3200 4400
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR048
U 1 1 5A760AC4
P 3200 4600
F 0 "#PWR048" H 3200 4350 50  0001 C CNN
F 1 "GND" H 3205 4427 50  0000 C CNN
F 2 "" H 3200 4600 50  0001 C CNN
F 3 "" H 3200 4600 50  0001 C CNN
	1    3200 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 4250 3200 4200
Wire Wire Line
	3200 4600 3200 4550
Wire Wire Line
	2850 4200 2850 4400
$Comp
L power:GND #PWR0105
U 1 1 5A760AD4
P 2850 4400
F 0 "#PWR0105" H 2850 4150 50  0001 C CNN
F 1 "GND" H 2850 4250 50  0000 C CNN
F 2 "" H 2850 4400 50  0000 C CNN
F 3 "" H 2850 4400 50  0000 C CNN
	1    2850 4400
	-1   0    0    -1  
$EndComp
$Comp
L device:C C35
U 1 1 5A760ADA
P 2850 4050
F 0 "C35" H 2800 3450 50  0000 L CNN
F 1 "10u" H 2800 3350 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2965 3959 50  0001 L CNN
F 3 "" H 2850 4050 50  0000 C CNN
	1    2850 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 4250 1650 4200
$Comp
L power:GND #PWR0108
U 1 1 5A760AE1
P 1650 4250
F 0 "#PWR0108" H 1650 4000 50  0001 C CNN
F 1 "GND" H 1650 4100 50  0000 C CNN
F 2 "" H 1650 4250 50  0000 C CNN
F 3 "" H 1650 4250 50  0000 C CNN
	1    1650 4250
	-1   0    0    -1  
$EndComp
$Comp
L device:C C33
U 1 1 5A760AE7
P 1650 4050
F 0 "C33" H 1600 3600 50  0000 L CNN
F 1 "4.7u 50V" H 1600 3500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1765 3959 50  0001 L CNN
F 3 "" H 1650 4050 50  0000 C CNN
	1    1650 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 3900 2850 3900
Connection ~ 1650 3900
Wire Wire Line
	1650 3900 2000 3900
$Comp
L regul:L7805 U12
U 1 1 5A760AF1
P 2300 3900
F 0 "U12" H 2300 4142 50  0000 C CNN
F 1 "L7803" H 2300 4051 50  0000 C CNN
F 2 "TO_SOT_Packages_THT:TO-220-3_Vertical" H 2325 3750 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 2300 3850 50  0001 C CNN
	1    2300 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5A760AF7
P 2300 4200
F 0 "#PWR0111" H 2300 3950 50  0001 C CNN
F 1 "GND" H 2300 4050 50  0000 C CNN
F 2 "" H 2300 4200 50  0000 C CNN
F 3 "" H 2300 4200 50  0000 C CNN
	1    2300 4200
	-1   0    0    -1  
$EndComp
Connection ~ 2850 3900
$Comp
L device:D_Zener_Small D37
U 1 1 5A760B00
P 3600 4000
F 0 "D37" V 3554 4068 50  0000 L CNN
F 1 "ESD5Z5.0T1G" V 3650 4150 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-523" V 3600 4000 50  0001 C CNN
F 3 "https://en.wikipedia.org/wiki/Zener_diode" V 3600 4000 50  0001 C CNN
	1    3600 4000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5A760B06
P 3600 4100
F 0 "#PWR0112" H 3600 3850 50  0001 C CNN
F 1 "GND" H 3600 3950 50  0000 C CNN
F 2 "" H 3600 4100 50  0001 C CNN
F 3 "" H 3600 4100 50  0001 C CNN
	1    3600 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 3900 3200 3900
Connection ~ 3200 3900
$Comp
L DeeComponents:TPS54202 U13
U 1 1 5A760B0F
P 2400 5550
F 0 "U13" H 2400 6017 50  0000 C CNN
F 1 "TPS54202" H 2400 5926 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-6" H 2400 5050 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/tps54202.pdf" H 2000 6000 50  0001 C CNN
	1    2400 5550
	1    0    0    -1  
$EndComp
NoConn ~ 1900 5550
$Comp
L power:GND #PWR0113
U 1 1 5A760B16
P 2400 5950
F 0 "#PWR0113" H 2400 5700 50  0001 C CNN
F 1 "GND" H 2400 5800 50  0000 C CNN
F 2 "" H 2400 5950 50  0000 C CNN
F 3 "" H 2400 5950 50  0000 C CNN
	1    2400 5950
	-1   0    0    -1  
$EndComp
$Comp
L device:C C36
U 1 1 5A760B1C
P 3200 5350
F 0 "C36" V 3400 5450 50  0000 L CNN
F 1 "0.1u 50V" V 3350 5350 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3315 5259 50  0001 L CNN
F 3 "" H 3200 5350 50  0000 C CNN
	1    3200 5350
	0    -1   -1   0   
$EndComp
$Comp
L device:L_Small L2
U 1 1 5A760B22
P 3550 5550
F 0 "L2" V 3372 5550 50  0000 C CNN
F 1 "22u" V 3463 5550 50  0000 C CNN
F 2 "components:Choke_SMD_7.3x7.3_H3.5_handsoldering" H 3550 5550 50  0001 C CNN
F 3 "" H 3550 5550 50  0001 C CNN
	1    3550 5550
	0    1    1    0   
$EndComp
$Comp
L device:C C37
U 1 1 5A760B28
P 3850 5700
F 0 "C37" H 3950 5550 50  0000 L CNN
F 1 "10u" H 3950 5450 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 3965 5609 50  0001 L CNN
F 3 "" H 3850 5700 50  0000 C CNN
	1    3850 5700
	1    0    0    -1  
$EndComp
$Comp
L device:R R68
U 1 1 5A760B2E
P 4500 6050
F 0 "R68" H 4570 6096 50  0000 L CNN
F 1 "100k" H 4570 6005 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4430 6050 50  0001 C CNN
F 3 "" H 4500 6050 50  0001 C CNN
	1    4500 6050
	1    0    0    -1  
$EndComp
$Comp
L device:C C38
U 1 1 5A760B34
P 4300 6050
F 0 "C38" H 4150 6150 50  0000 L CNN
F 1 "75p" H 4150 5950 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Disc_D3.0mm_W1.6mm_P2.50mm" H 4415 5959 50  0001 L CNN
F 3 "" H 4300 6050 50  0000 C CNN
	1    4300 6050
	1    0    0    -1  
$EndComp
$Comp
L device:R R70
U 1 1 5A760B3A
P 4500 6750
F 0 "R70" H 4570 6796 50  0000 L CNN
F 1 "2.2k" H 4570 6705 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4430 6750 50  0001 C CNN
F 3 "" H 4500 6750 50  0001 C CNN
	1    4500 6750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5A760B40
P 4500 7300
F 0 "#PWR0114" H 4500 7050 50  0001 C CNN
F 1 "GND" H 4500 7150 50  0000 C CNN
F 2 "" H 4500 7300 50  0000 C CNN
F 3 "" H 4500 7300 50  0000 C CNN
	1    4500 7300
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2900 5350 3050 5350
Wire Wire Line
	3350 5350 3350 5550
Wire Wire Line
	3350 5550 3450 5550
Wire Wire Line
	3350 5550 2900 5550
Connection ~ 3350 5550
Wire Wire Line
	3650 5550 3850 5550
Wire Wire Line
	3850 5550 4500 5550
Connection ~ 3850 5550
Wire Wire Line
	4300 5900 4500 5900
Connection ~ 4500 5900
Wire Wire Line
	4500 6200 4500 6250
Wire Wire Line
	4500 6250 4300 6250
Wire Wire Line
	4300 6250 4300 6200
Connection ~ 4500 6250
Wire Wire Line
	4500 6250 4500 6300
Wire Wire Line
	4300 6250 3250 6250
Wire Wire Line
	3250 6250 3250 5750
Wire Wire Line
	3250 5750 2900 5750
Connection ~ 4300 6250
Wire Wire Line
	4500 6900 4500 7000
$Comp
L power:GND #PWR0115
U 1 1 5A760B60
P 3850 5850
F 0 "#PWR0115" H 3850 5600 50  0001 C CNN
F 1 "GND" H 3850 5700 50  0000 C CNN
F 2 "" H 3850 5850 50  0000 C CNN
F 3 "" H 3850 5850 50  0000 C CNN
	1    3850 5850
	-1   0    0    -1  
$EndComp
$Comp
L device:R R69
U 1 1 5A760B66
P 4500 6450
F 0 "R69" H 4570 6496 50  0000 L CNN
F 1 "10k" H 4570 6405 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4430 6450 50  0001 C CNN
F 3 "" H 4500 6450 50  0001 C CNN
	1    4500 6450
	1    0    0    -1  
$EndComp
Text Notes 4800 6600 0    50   ~ 0
Vout = 0.596*((100000.0)/(10000.0+2200.0+1000.0)+1)\n=> 5.111151515151515\n
Wire Wire Line
	4500 5550 4500 5900
$Comp
L device:R R71
U 1 1 5A760B6F
P 4500 7150
F 0 "R71" H 4570 7196 50  0000 L CNN
F 1 "1k" H 4570 7105 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4430 7150 50  0001 C CNN
F 3 "" H 4500 7150 50  0001 C CNN
	1    4500 7150
	1    0    0    -1  
$EndComp
$Comp
L device:C C34
U 1 1 5A760B75
P 1650 5550
F 0 "C34" H 1500 5650 50  0000 L CNN
F 1 "1u" H 1500 5450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1765 5459 50  0001 L CNN
F 3 "" H 1650 5550 50  0000 C CNN
	1    1650 5550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5A760B7B
P 1650 5700
F 0 "#PWR0116" H 1650 5450 50  0001 C CNN
F 1 "GND" H 1650 5550 50  0000 C CNN
F 2 "" H 1650 5700 50  0000 C CNN
F 3 "" H 1650 5700 50  0000 C CNN
	1    1650 5700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1250 5350 1650 5350
Wire Wire Line
	1650 5400 1650 5350
Connection ~ 1650 5350
Wire Wire Line
	1650 5350 1900 5350
Text Label 2950 5550 0    50   ~ 0
VSW2
Text HLabel 1000 4850 0    50   Input ~ 0
AUX_VIN
Text HLabel 5000 5000 2    50   Input ~ 0
AUX_VOUT5V
Wire Wire Line
	2850 3900 3200 3900
Wire Wire Line
	3600 3900 4500 3900
Wire Wire Line
	4500 3900 4500 5000
Wire Wire Line
	4500 5000 5000 5000
Connection ~ 3600 3900
Wire Wire Line
	4500 5550 4500 5000
Connection ~ 4500 5550
Connection ~ 4500 5000
Wire Wire Line
	1250 5350 1250 4850
Wire Wire Line
	1250 3900 1650 3900
Wire Wire Line
	1000 4850 1250 4850
Connection ~ 1250 4850
Wire Wire Line
	1250 4850 1250 3900
$EndSCHEMATC
