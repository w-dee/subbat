EESchema Schematic File Version 4
LIBS:subbat-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 10 14
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
L DeeComponents:LT6105 U7
U 1 1 5A848B75
P 3400 2050
AR Path="/5A795145/5A848B75" Ref="U7"  Part="1" 
AR Path="/5A6A936D/5A848B75" Ref="U5"  Part="1" 
AR Path="/5A73ABE3/5A848B75" Ref="U6"  Part="1" 
AR Path="/5A7D5B3C/5A848B75" Ref="U8"  Part="1" 
F 0 "U8" H 3500 1900 50  0000 L CNN
F 1 "LT6105" H 3400 1800 50  0000 L CNN
F 2 "components:MSOP-8_3x3mm_Pitch0.65mm-thin" H 3500 1650 50  0001 C CNN
F 3 "" H 3450 2250 50  0001 C CNN
	1    3400 2050
	1    0    0    -1  
$EndComp
$Comp
L device:R R47
U 1 1 5A6AF967
P 2950 1950
AR Path="/5A795145/5A6AF967" Ref="R47"  Part="1" 
AR Path="/5A6A936D/5A6AF967" Ref="R35"  Part="1" 
AR Path="/5A73ABE3/5A6AF967" Ref="R41"  Part="1" 
AR Path="/5A7D5B3C/5A6AF967" Ref="R53"  Part="1" 
F 0 "R53" H 3020 1996 50  0000 L CNN
F 1 "120R" H 3020 1905 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2880 1950 50  0001 C CNN
F 3 "" H 2950 1950 50  0001 C CNN
	1    2950 1950
	0    -1   -1   0   
$EndComp
$Comp
L device:R R48
U 1 1 5A848B7A
P 2950 2150
AR Path="/5A795145/5A848B7A" Ref="R48"  Part="1" 
AR Path="/5A6A936D/5A848B7A" Ref="R36"  Part="1" 
AR Path="/5A73ABE3/5A848B7A" Ref="R42"  Part="1" 
AR Path="/5A7D5B3C/5A848B7A" Ref="R54"  Part="1" 
F 0 "R54" H 3020 2196 50  0000 L CNN
F 1 "120R" H 3020 2105 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2880 2150 50  0001 C CNN
F 3 "" H 2950 2150 50  0001 C CNN
	1    2950 2150
	0    -1   1    0   
$EndComp
$Comp
L device:C_Small C22
U 1 1 5A848B80
P 3500 1650
AR Path="/5A795145/5A848B80" Ref="C22"  Part="1" 
AR Path="/5A6A936D/5A848B80" Ref="C20"  Part="1" 
AR Path="/5A73ABE3/5A848B80" Ref="C21"  Part="1" 
AR Path="/5A7D5B3C/5A848B80" Ref="C23"  Part="1" 
F 0 "C23" H 3600 1650 50  0000 L CNN
F 1 "1u" H 3600 1550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3500 1650 50  0001 C CNN
F 3 "" H 3500 1650 50  0001 C CNN
	1    3500 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR072
U 1 1 5A848B83
P 3500 1750
AR Path="/5A795145/5A848B83" Ref="#PWR072"  Part="1" 
AR Path="/5A6A936D/5A848B83" Ref="#PWR079"  Part="1" 
AR Path="/5A73ABE3/5A848B83" Ref="#PWR069"  Part="1" 
AR Path="/5A7D5B3C/5A848B83" Ref="#PWR075"  Part="1" 
F 0 "#PWR075" H 3500 1500 50  0001 C CNN
F 1 "GND" H 3500 1600 50  0000 C CNN
F 2 "" H 3500 1750 50  0001 C CNN
F 3 "" H 3500 1750 50  0001 C CNN
	1    3500 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 1550 3300 1550
Wire Wire Line
	3300 1550 3300 1500
Wire Wire Line
	3300 1550 3300 1750
Connection ~ 3300 1550
$Comp
L power:GND #PWR073
U 1 1 5A6AF98D
P 3300 2650
AR Path="/5A795145/5A6AF98D" Ref="#PWR073"  Part="1" 
AR Path="/5A6A936D/5A6AF98D" Ref="#PWR078"  Part="1" 
AR Path="/5A73ABE3/5A6AF98D" Ref="#PWR070"  Part="1" 
AR Path="/5A7D5B3C/5A6AF98D" Ref="#PWR076"  Part="1" 
F 0 "#PWR076" H 3300 2400 50  0001 C CNN
F 1 "GND" H 3300 2500 50  0000 C CNN
F 2 "" H 3300 2650 50  0001 C CNN
F 3 "" H 3300 2650 50  0001 C CNN
	1    3300 2650
	1    0    0    -1  
$EndComp
$Comp
L device:R R49
U 1 1 5A848B8B
P 3700 2450
AR Path="/5A795145/5A848B8B" Ref="R49"  Part="1" 
AR Path="/5A6A936D/5A848B8B" Ref="R37"  Part="1" 
AR Path="/5A73ABE3/5A848B8B" Ref="R43"  Part="1" 
AR Path="/5A7D5B3C/5A848B8B" Ref="R55"  Part="1" 
F 0 "R55" H 3770 2496 50  0000 L CNN
F 1 "1k" H 3770 2405 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3630 2450 50  0001 C CNN
F 3 "" H 3700 2450 50  0001 C CNN
	1    3700 2450
	0    1    1    0   
$EndComp
Wire Wire Line
	3300 2350 3300 2450
Wire Wire Line
	3700 2050 4200 2050
Wire Wire Line
	4200 2450 4200 2050
Wire Wire Line
	3550 2450 3300 2450
Connection ~ 3300 2450
Wire Wire Line
	3300 2450 3300 2650
$Comp
L device:R R50
U 1 1 5A848B8C
P 4000 2450
AR Path="/5A795145/5A848B8C" Ref="R50"  Part="1" 
AR Path="/5A6A936D/5A848B8C" Ref="R38"  Part="1" 
AR Path="/5A73ABE3/5A848B8C" Ref="R44"  Part="1" 
AR Path="/5A7D5B3C/5A848B8C" Ref="R56"  Part="1" 
F 0 "R56" H 4070 2496 50  0000 L CNN
F 1 "120R" H 4070 2405 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3930 2450 50  0001 C CNN
F 3 "" H 4000 2450 50  0001 C CNN
	1    4000 2450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4150 2450 4200 2450
Text HLabel 2150 1450 2    50   Input ~ 0
LINE_P
Text HLabel 1550 1450 0    50   Input ~ 0
LINE_N
Text HLabel 4350 2050 2    50   Input ~ 0
SENS_OUT
Wire Wire Line
	4350 2050 4200 2050
Connection ~ 4200 2050
$Comp
L device:R R45
U 1 1 5A848B8F
P 1850 1450
AR Path="/5A795145/5A848B8F" Ref="R45"  Part="1" 
AR Path="/5A6A936D/5A848B8F" Ref="R33"  Part="1" 
AR Path="/5A73ABE3/5A848B8F" Ref="R39"  Part="1" 
AR Path="/5A7D5B3C/5A848B8F" Ref="R51"  Part="1" 
F 0 "R51" V 2000 1500 50  0000 L CNN
F 1 "0.01R" V 1920 1405 50  0000 L CNN
F 2 "Resistors_SMD:R_2512" V 1780 1450 50  0001 C CNN
F 3 "" H 1850 1450 50  0001 C CNN
	1    1850 1450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2000 1500 2050 1500
Wire Wire Line
	2150 1500 2150 1450
Wire Wire Line
	2000 1500 2000 1450
Wire Wire Line
	2050 1500 2050 1950
Wire Wire Line
	2050 1950 2800 1950
Connection ~ 2050 1500
Wire Wire Line
	2050 1500 2150 1500
Wire Wire Line
	1700 1500 1600 1500
Wire Wire Line
	1600 1500 1600 1450
Wire Wire Line
	1600 1450 1550 1450
Wire Wire Line
	1700 1500 1700 1450
Wire Wire Line
	1600 1500 1600 2150
Wire Wire Line
	1600 2150 2800 2150
Connection ~ 1600 1500
$Comp
L power:+5V #PWR082
U 1 1 5A6FCF13
P 3300 1500
AR Path="/5A795145/5A6FCF13" Ref="#PWR082"  Part="1" 
AR Path="/5A6A936D/5A6FCF13" Ref="#PWR080"  Part="1" 
AR Path="/5A73ABE3/5A6FCF13" Ref="#PWR081"  Part="1" 
AR Path="/5A7D5B3C/5A6FCF13" Ref="#PWR083"  Part="1" 
F 0 "#PWR083" H 3300 1350 50  0001 C CNN
F 1 "+5V" H 3315 1673 50  0000 C CNN
F 2 "" H 3300 1500 50  0001 C CNN
F 3 "" H 3300 1500 50  0001 C CNN
	1    3300 1500
	1    0    0    -1  
$EndComp
Text Notes 5250 2500 0    50   ~ 0
R_SENSE = 0.01R then\n10A => V_SENSE = 0.1V, Gain = (1k+120)/120 thus\nICHARGE_SENS=0.9333333333333333\n10mA => V_SENSE = 0.0001V (=Voffset of LT6105),\nICHARGE_SENS=0.0009333333333333333.\nThe maximum resolution is 10mA.
$EndSCHEMATC
