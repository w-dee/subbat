EESchema Schematic File Version 4
LIBS:subbat-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 12 14
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
F 2 "Diodes_SMD:D_SOD-123" H 2950 1200 50  0001 C CNN
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
F 2 "Diodes_SMD:D_SOD-123" H 2950 1550 50  0001 C CNN
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
L power:+3.3V #PWR077
U 1 1 5A6F5DF5
P 5000 1150
F 0 "#PWR077" H 5000 1000 50  0001 C CNN
F 1 "+3.3V" H 5015 1323 50  0000 C CNN
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
F 1 "4.7u" H 3550 800 50  0000 L CNN
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
$Comp
L device:D_Zener_Small D33
U 1 1 5A717C5D
P 5550 1300
F 0 "D33" V 5504 1368 50  0000 L CNN
F 1 "ESD5Z3.3T1G" V 5600 1450 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-523" V 5550 1300 50  0001 C CNN
F 3 "https://en.wikipedia.org/wiki/Zener_diode" V 5550 1300 50  0001 C CNN
	1    5550 1300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5A717C64
P 5550 1400
F 0 "#PWR0102" H 5550 1150 50  0001 C CNN
F 1 "GND" H 5550 1250 50  0000 C CNN
F 2 "" H 5550 1400 50  0001 C CNN
F 3 "" H 5550 1400 50  0001 C CNN
	1    5550 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 1200 5150 1200
Connection ~ 5150 1200
$EndSCHEMATC