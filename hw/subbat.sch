EESchema Schematic File Version 4
LIBS:subbat-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 16
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
L atmel:ATMEGA328P-PU U1
U 1 1 5A6064D9
P 2400 6300
F 0 "U1" H 2450 7667 50  0000 C CNN
F 1 "ATMEGA328P-PU" H 2450 7576 50  0000 C CNN
F 2 "Housings_DIP:DIP-28_W7.62mm_Socket" H 2400 6300 50  0001 C CIN
F 3 "" H 2400 6300 50  0001 C CNN
	1    2400 6300
	1    0    0    -1  
$EndComp
$Sheet
S 2850 1400 700  300 
U 5A615103
F0 "BiDiSwitch1" 50
F1 "BiDiSwitch.sch" 50
F2 "P1" B L 2850 1500 50 
F3 "P2" B R 3550 1500 50 
F4 "Control" B L 2850 1600 50 
$EndSheet
$Comp
L Connector:Conn_01x02 J1
U 1 1 5A619E8E
P 650 1600
F 0 "J1" H 570 1275 50  0000 C CNN
F 1 "Conn_01x02" H 570 1366 50  0000 C CNN
F 2 "Terminal_Blocks:TerminalBlock_bornier-2_P5.08mm" H 650 1600 50  0001 C CNN
F 3 "~" H 650 1600 50  0001 C CNN
	1    650  1600
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5A61A06C
P 850 1600
F 0 "#PWR01" H 850 1350 50  0001 C CNN
F 1 "GND" H 855 1427 50  0000 C CNN
F 2 "" H 850 1600 50  0001 C CNN
F 3 "" H 850 1600 50  0001 C CNN
	1    850  1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	850  1500 1050 1500
Text Label 1150 1500 0    50   ~ 0
VS
$Comp
L Connector:Conn_01x02 J2
U 1 1 5A61A16D
P 3700 850
F 0 "J2" H 3620 525 50  0000 C CNN
F 1 "Conn_01x02" H 3620 616 50  0000 C CNN
F 2 "Terminal_Blocks:TerminalBlock_bornier-2_P5.08mm" H 3700 850 50  0001 C CNN
F 3 "~" H 3700 850 50  0001 C CNN
	1    3700 850 
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5A61A173
P 3700 1050
F 0 "#PWR08" H 3700 800 50  0001 C CNN
F 1 "GND" H 3705 877 50  0000 C CNN
F 2 "" H 3700 1050 50  0001 C CNN
F 3 "" H 3700 1050 50  0001 C CNN
	1    3700 1050
	0    1    1    0   
$EndComp
Text Label 3750 1450 1    50   ~ 0
CCCV_OUT
$Comp
L Connector:Conn_01x02 J3
U 1 1 5A61A1F9
P 4300 850
F 0 "J3" H 4220 525 50  0000 C CNN
F 1 "Conn_01x02" H 4220 616 50  0000 C CNN
F 2 "Terminal_Blocks:TerminalBlock_bornier-2_P5.08mm" H 4300 850 50  0001 C CNN
F 3 "~" H 4300 850 50  0001 C CNN
	1    4300 850 
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5A61A1FF
P 4300 1050
F 0 "#PWR09" H 4300 800 50  0001 C CNN
F 1 "GND" H 4305 877 50  0000 C CNN
F 2 "" H 4300 1050 50  0001 C CNN
F 3 "" H 4300 1050 50  0001 C CNN
	1    4300 1050
	0    1    1    0   
$EndComp
$Sheet
S 5750 1350 700  300 
U 5A61A20F
F0 "BiDiSwitch2" 50
F1 "BiDiSwitch.sch" 50
F2 "P1" B L 5750 1450 50 
F3 "P2" B R 6450 1450 50 
F4 "Control" B L 5750 1550 50 
$EndSheet
Text Label 3950 1250 0    50   ~ 0
CCCV_IN
$Comp
L Connector:Conn_01x02 J4
U 1 1 5A61A39B
P 7600 1550
F 0 "J4" H 7520 1225 50  0000 C CNN
F 1 "Conn_01x02" H 7520 1316 50  0000 C CNN
F 2 "Terminal_Blocks:TerminalBlock_bornier-2_P5.08mm" H 7600 1550 50  0001 C CNN
F 3 "~" H 7600 1550 50  0001 C CNN
	1    7600 1550
	1    0    0    1   
$EndComp
Wire Wire Line
	6450 1450 6950 1450
$Comp
L power:GND #PWR012
U 1 1 5A61A413
P 7400 1550
F 0 "#PWR012" H 7400 1300 50  0001 C CNN
F 1 "GND" H 7405 1377 50  0000 C CNN
F 2 "" H 7400 1550 50  0001 C CNN
F 3 "" H 7400 1550 50  0001 C CNN
	1    7400 1550
	1    0    0    -1  
$EndComp
Text Label 6750 1450 0    50   ~ 0
VA
$Comp
L Connector:Conn_01x02 J5
U 1 1 5A61A4DA
P 9000 2550
F 0 "J5" H 8920 2225 50  0000 C CNN
F 1 "Conn_01x02" H 8920 2316 50  0000 C CNN
F 2 "Terminal_Blocks:TerminalBlock_bornier-2_P5.08mm" H 9000 2550 50  0001 C CNN
F 3 "~" H 9000 2550 50  0001 C CNN
	1    9000 2550
	1    0    0    1   
$EndComp
$Sheet
S 5750 2000 700  300 
U 5A61A4FA
F0 "BiDiSwitch3" 50
F1 "BiDiSwitch.sch" 50
F2 "P1" B L 5750 2100 50 
F3 "P2" B R 6450 2100 50 
F4 "Control" B L 5750 2200 50 
$EndSheet
$Sheet
S 5750 2500 700  300 
U 5A61A52C
F0 "BiDiSwitch4" 50
F1 "BiDiSwitch.sch" 50
F2 "P1" B L 5750 2600 50 
F3 "P2" B R 6450 2600 50 
F4 "Control" B L 5750 2700 50 
$EndSheet
Wire Wire Line
	6450 2100 6950 2100
Wire Wire Line
	6950 2600 6450 2600
$Comp
L power:GND #PWR013
U 1 1 5A61A5E8
P 8800 2550
F 0 "#PWR013" H 8800 2300 50  0001 C CNN
F 1 "GND" H 8805 2377 50  0000 C CNN
F 2 "" H 8800 2550 50  0001 C CNN
F 3 "" H 8800 2550 50  0001 C CNN
	1    8800 2550
	1    0    0    -1  
$EndComp
Text Label 7000 2450 0    50   ~ 0
VOUT
Wire Wire Line
	5750 2100 5650 2100
Wire Wire Line
	5650 2100 5650 1900
Wire Wire Line
	5650 1900 6950 1900
Wire Wire Line
	6950 1900 6950 1450
Connection ~ 6950 1450
Wire Wire Line
	6950 1450 7200 1450
Wire Wire Line
	2750 2600 2750 1500
Connection ~ 2750 1500
Wire Wire Line
	4400 1050 4400 1250
Wire Wire Line
	4400 1250 3900 1250
$Comp
L device:C_Small C3
U 1 1 5A62C9E3
P 1300 5900
F 0 "C3" H 1300 5500 50  0000 L CNN
F 1 "1u" H 1300 5400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1300 5900 50  0001 C CNN
F 3 "" H 1300 5900 50  0001 C CNN
	1    1300 5900
	1    0    0    -1  
$EndComp
$Comp
L device:C_Small C2
U 1 1 5A62DCC7
P 1100 5900
F 0 "C2" H 1100 5500 50  0000 L CNN
F 1 "1u" H 1100 5400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1100 5900 50  0001 C CNN
F 3 "" H 1100 5900 50  0001 C CNN
	1    1100 5900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5A62EBB6
P 1100 6000
F 0 "#PWR05" H 1100 5750 50  0001 C CNN
F 1 "GND" H 1100 5850 50  0000 C CNN
F 2 "" H 1100 6000 50  0001 C CNN
F 3 "" H 1100 6000 50  0001 C CNN
	1    1100 6000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5A62F710
P 1300 6000
F 0 "#PWR06" H 1300 5750 50  0001 C CNN
F 1 "GND" H 1300 5850 50  0000 C CNN
F 2 "" H 1300 6000 50  0001 C CNN
F 3 "" H 1300 6000 50  0001 C CNN
	1    1300 6000
	1    0    0    -1  
$EndComp
$Comp
L device:C_Small C1
U 1 1 5A630107
P 900 5900
F 0 "C1" H 900 5500 50  0000 L CNN
F 1 "1u" H 900 5400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 900 5900 50  0001 C CNN
F 3 "" H 900 5900 50  0001 C CNN
	1    900  5900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5A6309FE
P 900 6000
F 0 "#PWR04" H 900 5750 50  0001 C CNN
F 1 "GND" H 900 5850 50  0000 C CNN
F 2 "" H 900 6000 50  0001 C CNN
F 3 "" H 900 6000 50  0001 C CNN
	1    900  6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 5800 1500 5800
Wire Wire Line
	1100 5800 1100 5500
Wire Wire Line
	1100 5500 1500 5500
Wire Wire Line
	900  5800 900  5200
Wire Wire Line
	900  5200 1100 5200
Wire Wire Line
	1100 5500 1100 5200
Connection ~ 1100 5500
Connection ~ 1100 5200
Wire Wire Line
	1100 5200 1500 5200
$Comp
L power:+5V #PWR03
U 1 1 5A634F28
P 900 5200
F 0 "#PWR03" H 900 5050 50  0001 C CNN
F 1 "+5V" H 915 5373 50  0000 C CNN
F 2 "" H 900 5200 50  0001 C CNN
F 3 "" H 900 5200 50  0001 C CNN
	1    900  5200
	1    0    0    -1  
$EndComp
Connection ~ 900  5200
$Comp
L power:GND #PWR07
U 1 1 5A635DFB
P 1450 7550
F 0 "#PWR07" H 1450 7300 50  0001 C CNN
F 1 "GND" H 1450 7400 50  0000 C CNN
F 2 "" H 1450 7550 50  0001 C CNN
F 3 "" H 1450 7550 50  0001 C CNN
	1    1450 7550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 7550 1450 7500
Wire Wire Line
	1450 7500 1500 7500
Wire Wire Line
	1450 7500 1450 7400
Wire Wire Line
	1450 7400 1500 7400
Connection ~ 1450 7500
$Comp
L Connector:Conn_01x06_Male J7
U 1 1 5A6390B9
P 7050 5300
F 0 "J7" H 7157 5678 50  0000 C CNN
F 1 "Conn_01x06_Male" H 7157 5587 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 7050 5300 50  0001 C CNN
F 3 "~" H 7050 5300 50  0001 C CNN
	1    7050 5300
	-1   0    0    -1  
$EndComp
Text Label 6300 5100 0    50   ~ 0
MISO
Text Label 6300 5300 0    50   ~ 0
SCK
Text Label 6300 5400 0    50   ~ 0
MOSI
Text Label 6300 5500 0    50   ~ 0
~RESET
Wire Wire Line
	6300 5100 6850 5100
Wire Wire Line
	6850 5300 6300 5300
Wire Wire Line
	6850 5400 6300 5400
Wire Wire Line
	6850 5500 6300 5500
$Comp
L power:GND #PWR016
U 1 1 5A641CA1
P 6850 5600
F 0 "#PWR016" H 6850 5350 50  0001 C CNN
F 1 "GND" H 6850 5450 50  0000 C CNN
F 2 "" H 6850 5600 50  0001 C CNN
F 3 "" H 6850 5600 50  0001 C CNN
	1    6850 5600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR015
U 1 1 5A642F29
P 6850 5200
F 0 "#PWR015" H 6850 5050 50  0001 C CNN
F 1 "+5V" V 6865 5328 50  0000 L CNN
F 2 "" H 6850 5200 50  0001 C CNN
F 3 "" H 6850 5200 50  0001 C CNN
	1    6850 5200
	0    -1   -1   0   
$EndComp
Text Label 3750 6650 2    50   ~ 0
~RESET
Wire Wire Line
	3750 6650 3400 6650
Text Label 3750 5500 2    50   ~ 0
MOSI
Wire Wire Line
	3750 5500 3400 5500
Text Label 3750 5600 2    50   ~ 0
MISO
Wire Wire Line
	3750 5600 3400 5600
Text Label 3750 5700 2    50   ~ 0
SCK
Wire Wire Line
	3750 5700 3400 5700
$Sheet
S 10500 3050 500  150 
U 5A653DCD
F0 "USB-DCP1" 50
F1 "USB-DCP.sch" 50
F2 "VBUS_COM" I L 10500 3150 50 
$EndSheet
$Sheet
S 10100 5150 500  150 
U 5A64BD77
F0 "WaterLevel" 50
F1 "Water-Level.sch" 50
$EndSheet
Wire Wire Line
	1050 1550 1050 1500
Connection ~ 1050 1500
Wire Wire Line
	1050 1500 1350 1500
$Comp
L device:D_Zener_Small D2
U 1 1 5A6549CD
P 7200 1550
F 0 "D2" V 7200 1350 50  0000 L CNN
F 1 "30V" V 7300 1400 50  0000 L CNN
F 2 "Diodes_SMD:D_SMA_Handsoldering" V 7200 1550 50  0001 C CNN
F 3 "https://en.wikipedia.org/wiki/Zener_diode" V 7200 1550 50  0001 C CNN
	1    7200 1550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5A6549D3
P 7200 1650
F 0 "#PWR010" H 7200 1400 50  0001 C CNN
F 1 "GND" H 7205 1477 50  0000 C CNN
F 2 "" H 7200 1650 50  0001 C CNN
F 3 "" H 7200 1650 50  0001 C CNN
	1    7200 1650
	1    0    0    -1  
$EndComp
Connection ~ 7200 1450
Wire Wire Line
	7200 1450 7400 1450
$Comp
L device:D_Zener_Small D3
U 1 1 5A656392
P 8600 2550
F 0 "D3" V 8600 2350 50  0000 L CNN
F 1 "30V" V 8700 2350 50  0000 L CNN
F 2 "Diodes_SMD:D_SMA_Handsoldering" V 8600 2550 50  0001 C CNN
F 3 "https://en.wikipedia.org/wiki/Zener_diode" V 8600 2550 50  0001 C CNN
	1    8600 2550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5A656398
P 8600 2650
F 0 "#PWR011" H 8600 2400 50  0001 C CNN
F 1 "GND" H 8605 2477 50  0000 C CNN
F 2 "" H 8600 2650 50  0001 C CNN
F 3 "" H 8600 2650 50  0001 C CNN
	1    8600 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 2450 8700 2450
$Comp
L power:GND #PWR02
U 1 1 5A65169D
P 1050 1750
F 0 "#PWR02" H 1050 1500 50  0001 C CNN
F 1 "GND" H 1055 1577 50  0000 C CNN
F 2 "" H 1050 1750 50  0001 C CNN
F 3 "" H 1050 1750 50  0001 C CNN
	1    1050 1750
	1    0    0    -1  
$EndComp
$Comp
L device:D_Zener_Small D1
U 1 1 5A64E7A3
P 1050 1650
F 0 "D1" V 1004 1718 50  0000 L CNN
F 1 "30V" V 1095 1718 50  0000 L CNN
F 2 "Diodes_SMD:D_SMA_Handsoldering" V 1050 1650 50  0001 C CNN
F 3 "https://en.wikipedia.org/wiki/Zener_diode" V 1050 1650 50  0001 C CNN
	1    1050 1650
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x02 J6
U 1 1 5A65D685
P 9000 3250
F 0 "J6" H 8920 2925 50  0000 C CNN
F 1 "Conn_01x02" H 8920 3016 50  0000 C CNN
F 2 "Terminal_Blocks:TerminalBlock_bornier-2_P5.08mm" H 9000 3250 50  0001 C CNN
F 3 "~" H 9000 3250 50  0001 C CNN
	1    9000 3250
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5A65D68B
P 8800 3250
F 0 "#PWR014" H 8800 3000 50  0001 C CNN
F 1 "GND" H 8805 3077 50  0000 C CNN
F 2 "" H 8800 3250 50  0001 C CNN
F 3 "" H 8800 3250 50  0001 C CNN
	1    8800 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 3150 8700 3150
Wire Wire Line
	8700 3150 8700 2450
Connection ~ 8700 2450
Wire Wire Line
	8700 2450 8800 2450
$Comp
L Connector:Conn_01x02 J8
U 1 1 5A662C89
P 9850 2100
F 0 "J8" H 9770 1775 50  0000 C CNN
F 1 "Conn_01x02" H 9770 1866 50  0000 C CNN
F 2 "Terminal_Blocks:TerminalBlock_bornier-2_P5.08mm" H 9850 2100 50  0001 C CNN
F 3 "~" H 9850 2100 50  0001 C CNN
	1    9850 2100
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5A662C8F
P 10050 2100
F 0 "#PWR020" H 10050 1850 50  0001 C CNN
F 1 "GND" H 10055 1927 50  0000 C CNN
F 2 "" H 10050 2100 50  0001 C CNN
F 3 "" H 10050 2100 50  0001 C CNN
	1    10050 2100
	-1   0    0    -1  
$EndComp
Text Label 10500 2000 2    50   ~ 0
VBUS_COM_IN
Text Label 10050 3150 0    50   ~ 0
VBUS_COM
Wire Wire Line
	10050 3150 10500 3150
$Sheet
S 10500 3350 500  150 
U 5A676C71
F0 "USB-DCP2" 50
F1 "USB-DCP.sch" 50
F2 "VBUS_COM" I L 10500 3450 50 
$EndSheet
Text Label 10050 3450 0    50   ~ 0
VBUS_COM
Wire Wire Line
	10050 3450 10500 3450
$Sheet
S 10500 3650 500  150 
U 5A678AED
F0 "USB-DCP3" 50
F1 "USB-DCP.sch" 50
F2 "VBUS_COM" I L 10500 3750 50 
$EndSheet
Text Label 10050 3750 0    50   ~ 0
VBUS_COM
Wire Wire Line
	10050 3750 10500 3750
$Sheet
S 10500 3950 500  150 
U 5A678AF2
F0 "USB-DCP4" 50
F1 "USB-DCP.sch" 50
F2 "VBUS_COM" I L 10500 4050 50 
$EndSheet
Text Label 10050 4050 0    50   ~ 0
VBUS_COM
Wire Wire Line
	10050 4050 10500 4050
Text Label 3750 7300 2    50   ~ 0
SW_VR1
Text Label 3750 7400 2    50   ~ 0
SW_VR2
Text Label 3750 7500 2    50   ~ 0
SW_BAT
Text Label 4100 5200 2    50   ~ 0
SW_VS
Text Label 4100 5300 2    50   ~ 0
~DEEP_SLEEP_EN
Text Label 4000 6050 2    50   ~ 0
VS_SENSE_ADC
Text Label 4000 6150 2    50   ~ 0
VA_SENSE_ADC
Wire Wire Line
	4100 5200 3400 5200
Wire Wire Line
	4100 5300 3400 5300
Wire Wire Line
	4000 6150 3400 6150
Wire Wire Line
	3750 7300 3400 7300
Wire Wire Line
	3400 7400 3750 7400
Wire Wire Line
	3750 7500 3400 7500
Text Label 6900 6250 3    50   ~ 0
VA_SENSE_ADC
$Comp
L device:R R23
U 1 1 5A67B40A
P 6550 6350
F 0 "R23" H 6620 6396 50  0000 L CNN
F 1 "10k" H 6620 6305 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6480 6350 50  0001 C CNN
F 3 "" H 6550 6350 50  0001 C CNN
	1    6550 6350
	1    0    0    -1  
$EndComp
$Comp
L device:R R24
U 1 1 5A67DE67
P 6550 6700
F 0 "R24" H 6620 6746 50  0000 L CNN
F 1 "10k" H 6620 6655 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6480 6700 50  0001 C CNN
F 3 "" H 6550 6700 50  0001 C CNN
	1    6550 6700
	1    0    0    -1  
$EndComp
$Comp
L device:R R25
U 1 1 5A68089F
P 6550 7050
F 0 "R25" H 6620 7096 50  0000 L CNN
F 1 "1k" H 6620 7005 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6480 7050 50  0001 C CNN
F 3 "" H 6550 7050 50  0001 C CNN
	1    6550 7050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR052
U 1 1 5A6832C2
P 6550 7250
F 0 "#PWR052" H 6550 7000 50  0001 C CNN
F 1 "GND" H 6550 7100 50  0000 C CNN
F 2 "" H 6550 7250 50  0001 C CNN
F 3 "" H 6550 7250 50  0001 C CNN
	1    6550 7250
	1    0    0    -1  
$EndComp
Text Label 5200 6200 0    50   ~ 0
VA_G
Wire Wire Line
	6550 6850 6550 6900
Wire Wire Line
	6550 7250 6550 7200
Wire Wire Line
	6550 6550 6550 6500
Text Label 4400 6350 3    50   ~ 0
VS_SENSE_ADC
Wire Wire Line
	4400 6350 4400 6950
$Comp
L device:R R20
U 1 1 5A6A6835
P 4550 6450
F 0 "R20" H 4620 6496 50  0000 L CNN
F 1 "10k" H 4620 6405 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4480 6450 50  0001 C CNN
F 3 "" H 4550 6450 50  0001 C CNN
	1    4550 6450
	1    0    0    -1  
$EndComp
$Comp
L device:R R21
U 1 1 5A6A683B
P 4550 6800
F 0 "R21" H 4620 6846 50  0000 L CNN
F 1 "10k" H 4620 6755 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4480 6800 50  0001 C CNN
F 3 "" H 4550 6800 50  0001 C CNN
	1    4550 6800
	1    0    0    -1  
$EndComp
$Comp
L device:R R22
U 1 1 5A6A6841
P 4550 7150
F 0 "R22" H 4620 7196 50  0000 L CNN
F 1 "1k" H 4620 7105 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4480 7150 50  0001 C CNN
F 3 "" H 4550 7150 50  0001 C CNN
	1    4550 7150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR051
U 1 1 5A6A6847
P 4550 7350
F 0 "#PWR051" H 4550 7100 50  0001 C CNN
F 1 "GND" H 4550 7200 50  0000 C CNN
F 2 "" H 4550 7350 50  0001 C CNN
F 3 "" H 4550 7350 50  0001 C CNN
	1    4550 7350
	1    0    0    -1  
$EndComp
Text Label 4900 6900 1    50   ~ 0
VS
Wire Wire Line
	4900 6900 4900 6300
Wire Wire Line
	4400 6950 4550 6950
Wire Wire Line
	4550 6950 4550 7000
Connection ~ 4550 6950
Wire Wire Line
	4550 7350 4550 7300
Wire Wire Line
	4550 6650 4550 6600
Wire Wire Line
	4550 6300 4900 6300
$Comp
L Connector:Conn_01x05_Female J14
U 1 1 5A6B09AA
P 8500 5400
F 0 "J14" H 8527 5426 50  0000 L CNN
F 1 "Conn_01x05_Female" H 8527 5335 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05_Pitch2.54mm" H 8500 5400 50  0001 C CNN
F 3 "~" H 8500 5400 50  0001 C CNN
	1    8500 5400
	1    0    0    -1  
$EndComp
Text Label 4000 7000 2    50   ~ 0
GPIO_AUX1
Text Label 4000 7100 2    50   ~ 0
GPIO_AUX2
Text Label 4000 7200 2    50   ~ 0
GPIO_AUX3
Wire Wire Line
	4000 7000 3400 7000
Wire Wire Line
	3400 7100 4000 7100
Wire Wire Line
	4000 7200 3400 7200
Text Label 7700 5300 0    50   ~ 0
GPIO_AUX1
Text Label 7700 5400 0    50   ~ 0
GPIO_AUX2
Text Label 7700 5500 0    50   ~ 0
GPIO_AUX3
Wire Wire Line
	7700 5300 8300 5300
Wire Wire Line
	8300 5400 7700 5400
Wire Wire Line
	7700 5500 8300 5500
$Comp
L power:GND #PWR053
U 1 1 5A6CF037
P 8100 5000
F 0 "#PWR053" H 8100 4750 50  0001 C CNN
F 1 "GND" H 8100 4850 50  0000 C CNN
F 2 "" H 8100 5000 50  0001 C CNN
F 3 "" H 8100 5000 50  0001 C CNN
	1    8100 5000
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR054
U 1 1 5A6D2B3D
P 8300 5600
F 0 "#PWR054" H 8300 5450 50  0001 C CNN
F 1 "+5V" V 8315 5728 50  0000 L CNN
F 2 "" H 8300 5600 50  0001 C CNN
F 3 "" H 8300 5600 50  0001 C CNN
	1    8300 5600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8300 5200 8250 5200
Wire Wire Line
	8250 5200 8250 5000
Wire Wire Line
	8250 5000 8100 5000
$Sheet
S 10100 5750 500  150 
U 5A6E3BC6
F0 "SingleButtonRemocon" 50
F1 "SingleButtonRemocon.sch" 50
$EndSheet
$Comp
L Connector:Conn_01x06_Male J17
U 1 1 5A658D55
P 5700 5300
F 0 "J17" H 5807 5678 50  0000 C CNN
F 1 "Conn_01x06_Male" H 5807 5587 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 5700 5300 50  0001 C CNN
F 3 "~" H 5700 5300 50  0001 C CNN
	1    5700 5300
	-1   0    0    -1  
$EndComp
Text Label 4650 5300 0    50   ~ 0
TX
Text Label 4650 5400 0    50   ~ 0
RX
$Comp
L power:GND #PWR062
U 1 1 5A668264
P 5350 5500
F 0 "#PWR062" H 5350 5250 50  0001 C CNN
F 1 "GND" H 5350 5350 50  0000 C CNN
F 2 "" H 5350 5500 50  0001 C CNN
F 3 "" H 5350 5500 50  0001 C CNN
	1    5350 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 5500 5350 5500
$Comp
L device:R R28
U 1 1 5A680297
P 5100 5300
F 0 "R28" H 5170 5346 50  0000 L CNN
F 1 "120R" H 5170 5255 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5030 5300 50  0001 C CNN
F 3 "" H 5100 5300 50  0001 C CNN
	1    5100 5300
	0    -1   -1   0   
$EndComp
$Comp
L device:R R29
U 1 1 5A68C777
P 5100 5400
F 0 "R29" H 5170 5446 50  0000 L CNN
F 1 "120R" H 5170 5355 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5030 5400 50  0001 C CNN
F 3 "" H 5100 5400 50  0001 C CNN
	1    5100 5400
	0    -1   1    0   
$EndComp
Wire Wire Line
	5250 5300 5500 5300
Wire Wire Line
	5500 5400 5250 5400
Wire Wire Line
	4650 5300 4950 5300
Wire Wire Line
	4650 5400 4950 5400
NoConn ~ 5500 5100
NoConn ~ 5500 5200
NoConn ~ 5500 5600
Text Label 3750 6900 2    50   ~ 0
TX
Text Label 3750 6800 2    50   ~ 0
RX
Wire Wire Line
	3750 6800 3400 6800
Wire Wire Line
	3400 6900 3750 6900
$Comp
L device:CP1_Small C19
U 1 1 5A6DAE22
P 6950 2900
F 0 "C19" H 6750 2900 50  0000 L CNN
F 1 "1600u 35V" H 6450 2800 50  0000 L CNN
F 2 "Capacitors_ThroughHole:CP_Radial_D13.0mm_P5.00mm" H 6950 2900 50  0001 C CNN
F 3 "" H 6950 2900 50  0001 C CNN
	1    6950 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 2800 6950 2600
Connection ~ 6950 2600
$Comp
L power:GND #PWR063
U 1 1 5A6DF66D
P 6950 3000
F 0 "#PWR063" H 6950 2750 50  0001 C CNN
F 1 "GND" H 6955 2827 50  0000 C CNN
F 2 "" H 6950 3000 50  0001 C CNN
F 3 "" H 6950 3000 50  0001 C CNN
	1    6950 3000
	1    0    0    -1  
$EndComp
Text Label 3250 1900 2    50   ~ 0
SW_VR1
Text Label 6100 1850 2    50   ~ 0
SW_VR2
Text Label 5400 2200 0    50   ~ 0
SW_BAT
Text Label 5400 2700 0    50   ~ 0
SW_VS
Wire Wire Line
	5750 1550 5700 1550
Wire Wire Line
	5700 1550 5700 1850
Wire Wire Line
	5700 1850 6100 1850
Wire Wire Line
	2800 1600 2800 1900
Wire Wire Line
	2800 1900 3250 1900
$Comp
L device:Q_PMOS_GSD Q17
U 1 1 5A68A7D5
P 5700 6300
F 0 "Q17" V 6043 6300 50  0000 C CNN
F 1 "Q_PMOS_GSD" V 5952 6300 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 5900 6400 50  0001 C CNN
F 3 "" H 5700 6300 50  0001 C CNN
	1    5700 6300
	0    1    -1   0   
$EndComp
$Comp
L device:Q_NPN_BEC Q16
U 1 1 5A68A7DB
P 5600 6850
F 0 "Q16" H 5791 6896 50  0000 L CNN
F 1 "Q_NPN_BEC" H 5791 6805 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 5800 6950 50  0001 C CNN
F 3 "" H 5600 6850 50  0001 C CNN
	1    5600 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 7150 5700 7050
$Comp
L power:GND #PWR067
U 1 1 5A68A7E6
P 5700 7150
F 0 "#PWR067" H 5700 6900 50  0001 C CNN
F 1 "GND" H 5705 6977 50  0000 C CNN
F 2 "" H 5700 7150 50  0001 C CNN
F 3 "" H 5700 7150 50  0001 C CNN
	1    5700 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 6500 5700 6550
Wire Wire Line
	5200 6200 5400 6200
Wire Wire Line
	6550 6900 6900 6900
Wire Wire Line
	6900 6250 6900 6900
Connection ~ 6550 6900
Wire Wire Line
	6550 6200 5900 6200
$Comp
L device:R R32
U 1 1 5A6ECD92
P 5400 6400
F 0 "R32" H 5470 6446 50  0000 L CNN
F 1 "470k" H 5470 6355 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5330 6400 50  0001 C CNN
F 3 "" H 5400 6400 50  0001 C CNN
	1    5400 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 6250 5400 6200
Connection ~ 5400 6200
Wire Wire Line
	5400 6200 5500 6200
Wire Wire Line
	5400 6550 5700 6550
Connection ~ 5700 6550
Wire Wire Line
	5700 6550 5700 6650
$Comp
L device:R R31
U 1 1 5A6FE4CB
P 5300 6700
F 0 "R31" H 5370 6746 50  0000 L CNN
F 1 "10k" H 5370 6655 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5230 6700 50  0001 C CNN
F 3 "" H 5300 6700 50  0001 C CNN
	1    5300 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 6850 5400 6850
Text Label 4100 5400 2    50   ~ 0
VA_SENS_EN
Wire Wire Line
	4100 5400 3400 5400
Text Label 5150 6400 3    50   ~ 0
VA_SENS_EN
Wire Wire Line
	5150 6400 5150 7100
Wire Wire Line
	5150 7100 5200 7100
Wire Wire Line
	5200 7100 5200 6550
Wire Wire Line
	5200 6550 5300 6550
Text Label 4000 6250 2    50   ~ 0
ICHARGE_SENS
Wire Wire Line
	4000 6050 3400 6050
Wire Wire Line
	4000 6250 3400 6250
Text Notes 7500 7050 0    50   ~ 0
R_SENSE = 0.01R then\n10A => V_SENSE = 0.1V, Gain = (1k+120)/120 thus\nICHARGE_SENS=0.9333333333333333\n10mA => V_SENSE = 0.0001V (=Voffset of LT6105),\nICHARGE_SENS=0.0009333333333333333.\nThe maximum resolution is 10mA.
Wire Wire Line
	10050 2000 10500 2000
Connection ~ 8600 2450
Text Label 4000 6350 2    50   ~ 0
IOUT_SENS
Wire Wire Line
	3400 6350 4000 6350
Text Label 8450 2450 0    50   ~ 0
VOUT2
$Sheet
S 10150 4450 950  400 
U 5A689825
F0 "Power5V" 50
F1 "Power5V.sch" 50
F2 "VA" I L 10150 4500 50 
F3 "VS" I L 10150 4600 50 
F4 "~DEEP_SLEEP_EN" I L 10150 4700 50 
F5 "VA_G" I L 10150 4800 50 
$EndSheet
$Sheet
S 4400 1350 750  300 
U 5A6A936D
F0 "CurrentSens_ICHARGE" 50
F1 "CurrentSens.sch" 50
F2 "LINE_P" I L 4400 1450 50 
F3 "LINE_N" I R 5150 1450 50 
F4 "SENS_OUT" I L 4400 1550 50 
$EndSheet
Wire Wire Line
	5150 1450 5750 1450
Wire Wire Line
	4400 1450 3900 1450
Wire Wire Line
	3900 1450 3900 1250
Text Label 3800 1550 0    50   ~ 0
ICHARGE_SENS
Wire Wire Line
	3800 1550 4400 1550
Text Label 5350 1450 0    50   ~ 0
CCCV_IN2
Wire Wire Line
	5400 2200 5750 2200
Wire Wire Line
	5400 2700 5750 2700
Wire Wire Line
	6950 2100 6950 2450
$Sheet
S 7350 2350 750  300 
U 5A73ABE3
F0 "CurrentSens_IOUT" 50
F1 "CurrentSens.sch" 50
F2 "LINE_P" I L 7350 2450 50 
F3 "LINE_N" I R 8100 2450 50 
F4 "SENS_OUT" I L 7350 2550 50 
$EndSheet
Wire Wire Line
	8100 2450 8600 2450
Wire Wire Line
	7350 2450 6950 2450
Connection ~ 6950 2450
Wire Wire Line
	6950 2450 6950 2600
Text Label 7200 3000 1    50   ~ 0
IOUT_SENS
Wire Wire Line
	7200 3000 7200 2550
Wire Wire Line
	7200 2550 7350 2550
Wire Wire Line
	3550 1500 3750 1500
Wire Wire Line
	3750 1500 3750 1050
Wire Wire Line
	3750 1050 3800 1050
$Sheet
S 1350 1400 750  300 
U 5A795145
F0 "CurrentSens_ISYS" 50
F1 "CurrentSens.sch" 50
F2 "LINE_P" I L 1350 1500 50 
F3 "LINE_N" I R 2100 1500 50 
F4 "SENS_OUT" I L 1350 1600 50 
$EndSheet
Wire Wire Line
	2100 1500 2750 1500
Wire Wire Line
	2800 1600 2850 1600
Wire Wire Line
	2750 1500 2850 1500
Wire Wire Line
	2750 2600 5750 2600
Text Label 4000 6450 2    50   ~ 0
ISYS_SENS
Wire Wire Line
	3400 6450 4000 6450
Text Label 4000 6550 2    50   ~ 0
IUSB_SENS
Wire Wire Line
	3400 6550 4000 6550
Text Label 1750 1900 2    50   ~ 0
ISYS_SENS
Wire Wire Line
	1350 1600 1300 1600
Wire Wire Line
	1300 1600 1300 1900
Wire Wire Line
	1300 1900 1750 1900
$Sheet
S 10100 2400 750  300 
U 5A7D5B3C
F0 "CurrentSens_IUSB" 50
F1 "CurrentSens.sch" 50
F2 "LINE_P" I L 10100 2500 50 
F3 "LINE_N" I R 10850 2500 50 
F4 "SENS_OUT" I L 10100 2600 50 
$EndSheet
Text Label 9500 2500 0    50   ~ 0
VBUS_COM_IN
Wire Wire Line
	9500 2500 10100 2500
Text Label 11100 2950 1    50   ~ 0
VBUS_COM
Wire Wire Line
	11100 2950 11100 2500
Wire Wire Line
	10850 2500 11100 2500
Text Label 10550 2850 2    50   ~ 0
IUSB_SENS
Wire Wire Line
	10000 2850 10000 2600
Wire Wire Line
	10000 2600 10100 2600
Wire Wire Line
	10000 2850 10550 2850
Text Label 2350 1500 0    50   ~ 0
VS2
Text Notes 4050 7100 0    50   ~ 0
SCL
Text Notes 4050 7200 0    50   ~ 0
SDA
Text Label 9500 4700 0    50   ~ 0
~DEEP_SLEEP_EN
Wire Wire Line
	10150 4700 9500 4700
Text Label 9900 4500 0    50   ~ 0
VA
Text Label 9900 4600 0    50   ~ 0
VS2
Wire Wire Line
	9900 4500 10150 4500
Wire Wire Line
	9900 4600 10150 4600
Text Label 9900 4800 0    50   ~ 0
VA_G
Wire Wire Line
	9900 4800 10150 4800
$EndSCHEMATC
