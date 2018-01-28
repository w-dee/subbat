EESchema Schematic File Version 4
LIBS:subbat-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 16
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
L DeeComponents:AOD403 Q3
U 1 1 5A6151C6
P 2550 1750
AR Path="/5A615103/5A6151C6" Ref="Q3"  Part="1" 
AR Path="/5A61A20F/5A6151C6" Ref="Q6"  Part="1" 
AR Path="/5A61A4FA/5A6151C6" Ref="Q9"  Part="1" 
AR Path="/5A61A52C/5A6151C6" Ref="Q12"  Part="1" 
F 0 "Q12" V 2800 1750 50  0000 C CNN
F 1 "AOD403" V 2891 1750 50  0000 C CNN
F 2 "components:TO-263-2-with-holes" H 2800 1675 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf9540n.pdf" H 2550 1750 50  0001 L CNN
	1    2550 1750
	0    -1   -1   0   
$EndComp
$Comp
L DeeComponents:AOD403 Q5
U 1 1 5A6151CD
P 3600 1750
AR Path="/5A615103/5A6151CD" Ref="Q5"  Part="1" 
AR Path="/5A61A20F/5A6151CD" Ref="Q8"  Part="1" 
AR Path="/5A61A4FA/5A6151CD" Ref="Q11"  Part="1" 
AR Path="/5A61A52C/5A6151CD" Ref="Q14"  Part="1" 
F 0 "Q14" V 3850 1750 50  0000 C CNN
F 1 "AOD403" V 3941 1750 50  0000 C CNN
F 2 "components:TO-263-2-with-holes" H 3850 1675 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf9540n.pdf" H 3600 1750 50  0001 L CNN
	1    3600 1750
	0    1    -1   0   
$EndComp
$Comp
L device:R R6
U 1 1 5A6151D4
P 2850 1850
AR Path="/5A615103/5A6151D4" Ref="R6"  Part="1" 
AR Path="/5A61A20F/5A6151D4" Ref="R10"  Part="1" 
AR Path="/5A61A4FA/5A6151D4" Ref="R14"  Part="1" 
AR Path="/5A61A52C/5A6151D4" Ref="R18"  Part="1" 
F 0 "R18" H 2920 1896 50  0000 L CNN
F 1 "470k" H 2920 1805 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2780 1850 50  0001 C CNN
F 3 "" H 2850 1850 50  0001 C CNN
	1    2850 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 1650 2850 1650
Wire Wire Line
	2850 1700 2850 1650
Connection ~ 2850 1650
Wire Wire Line
	2550 1950 2550 2050
Wire Wire Line
	2550 2050 2850 2050
Wire Wire Line
	2850 2050 2850 2000
Wire Wire Line
	3600 2050 3600 1950
Text HLabel 2050 1650 0    50   BiDi ~ 0
P1
Text HLabel 4100 1650 2    50   BiDi ~ 0
P2
Wire Wire Line
	2350 1650 2050 1650
Wire Wire Line
	3800 1650 4100 1650
$Comp
L device:Q_NPN_BEC Q4
U 1 1 5A615C97
P 2900 2800
AR Path="/5A615103/5A615C97" Ref="Q4"  Part="1" 
AR Path="/5A61A20F/5A615C97" Ref="Q7"  Part="1" 
AR Path="/5A61A4FA/5A615C97" Ref="Q10"  Part="1" 
AR Path="/5A61A52C/5A615C97" Ref="Q13"  Part="1" 
F 0 "Q13" H 3091 2846 50  0000 L CNN
F 1 "Q_NPN_BEC" H 3091 2755 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 3100 2900 50  0001 C CNN
F 3 "" H 2900 2800 50  0001 C CNN
	1    2900 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 5A615DED
P 3000 3100
AR Path="/5A615103/5A615DED" Ref="#PWR023"  Part="1" 
AR Path="/5A61A20F/5A615DED" Ref="#PWR025"  Part="1" 
AR Path="/5A61A4FA/5A615DED" Ref="#PWR027"  Part="1" 
AR Path="/5A61A52C/5A615DED" Ref="#PWR029"  Part="1" 
F 0 "#PWR029" H 3000 2850 50  0001 C CNN
F 1 "GND" H 3005 2927 50  0000 C CNN
F 2 "" H 3000 3100 50  0001 C CNN
F 3 "" H 3000 3100 50  0001 C CNN
	1    3000 3100
	1    0    0    -1  
$EndComp
$Comp
L device:R R4
U 1 1 5A615E3D
P 2600 2950
AR Path="/5A615103/5A615E3D" Ref="R4"  Part="1" 
AR Path="/5A61A20F/5A615E3D" Ref="R8"  Part="1" 
AR Path="/5A61A4FA/5A615E3D" Ref="R12"  Part="1" 
AR Path="/5A61A52C/5A615E3D" Ref="R16"  Part="1" 
F 0 "R16" H 2670 2996 50  0000 L CNN
F 1 "10k" H 2670 2905 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2530 2950 50  0001 C CNN
F 3 "" H 2600 2950 50  0001 C CNN
	1    2600 2950
	1    0    0    -1  
$EndComp
Text HLabel 2600 3250 3    50   BiDi ~ 0
Control
Wire Wire Line
	2600 3250 2600 3150
Wire Wire Line
	2600 2800 2700 2800
Wire Wire Line
	3000 3100 3000 3000
$Comp
L device:R R5
U 1 1 5A615695
P 2750 3450
AR Path="/5A615103/5A615695" Ref="R5"  Part="1" 
AR Path="/5A61A20F/5A615695" Ref="R9"  Part="1" 
AR Path="/5A61A4FA/5A615695" Ref="R13"  Part="1" 
AR Path="/5A61A52C/5A615695" Ref="R17"  Part="1" 
F 0 "R17" H 2820 3496 50  0000 L CNN
F 1 "2.2k" H 2820 3405 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2680 3450 50  0001 C CNN
F 3 "" H 2750 3450 50  0001 C CNN
	1    2750 3450
	1    0    0    -1  
$EndComp
$Comp
L device:LED D7
U 1 1 5A61597B
P 2750 3800
AR Path="/5A615103/5A61597B" Ref="D7"  Part="1" 
AR Path="/5A61A20F/5A61597B" Ref="D9"  Part="1" 
AR Path="/5A61A4FA/5A61597B" Ref="D11"  Part="1" 
AR Path="/5A61A52C/5A61597B" Ref="D13"  Part="1" 
F 0 "D13" V 2788 3683 50  0000 R CNN
F 1 "LED" V 2697 3683 50  0000 R CNN
F 2 "LEDs:LED_D3.0mm" H 2750 3800 50  0001 C CNN
F 3 "~" H 2750 3800 50  0001 C CNN
	1    2750 3800
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5A615C2D
P 2750 4000
AR Path="/5A615103/5A615C2D" Ref="#PWR022"  Part="1" 
AR Path="/5A61A20F/5A615C2D" Ref="#PWR024"  Part="1" 
AR Path="/5A61A4FA/5A615C2D" Ref="#PWR026"  Part="1" 
AR Path="/5A61A52C/5A615C2D" Ref="#PWR028"  Part="1" 
F 0 "#PWR028" H 2750 3750 50  0001 C CNN
F 1 "GND" H 2755 3827 50  0000 C CNN
F 2 "" H 2750 4000 50  0001 C CNN
F 3 "" H 2750 4000 50  0001 C CNN
	1    2750 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 3650 2750 3600
Wire Wire Line
	2750 4000 2750 3950
Wire Wire Line
	2600 3150 2750 3150
Wire Wire Line
	2750 3150 2750 3300
Connection ~ 2600 3150
Wire Wire Line
	2600 3150 2600 3100
$Comp
L device:D_Zener_Small D8
U 1 1 5A655DCB
P 3200 1850
AR Path="/5A615103/5A655DCB" Ref="D8"  Part="1" 
AR Path="/5A61A20F/5A655DCB" Ref="D10"  Part="1" 
AR Path="/5A61A4FA/5A655DCB" Ref="D12"  Part="1" 
AR Path="/5A61A52C/5A655DCB" Ref="D14"  Part="1" 
F 0 "D14" V 3154 1918 50  0000 L CNN
F 1 "12V" V 3245 1918 50  0000 L CNN
F 2 "Diodes_SMD:D_1206" V 3200 1850 50  0001 C CNN
F 3 "https://en.wikipedia.org/wiki/Zener_diode" V 3200 1850 50  0001 C CNN
	1    3200 1850
	0    1    1    0   
$EndComp
Wire Wire Line
	2850 1650 3200 1650
Wire Wire Line
	3200 1750 3200 1650
Connection ~ 3200 1650
Wire Wire Line
	3200 1650 3400 1650
Wire Wire Line
	3200 1950 3200 2050
Wire Wire Line
	3200 2050 3600 2050
Wire Wire Line
	3200 2050 3000 2050
Connection ~ 3200 2050
Connection ~ 2850 2050
$Comp
L device:R R7
U 1 1 5A6575C9
P 3000 2250
AR Path="/5A615103/5A6575C9" Ref="R7"  Part="1" 
AR Path="/5A61A20F/5A6575C9" Ref="R11"  Part="1" 
AR Path="/5A61A4FA/5A6575C9" Ref="R15"  Part="1" 
AR Path="/5A61A52C/5A6575C9" Ref="R19"  Part="1" 
F 0 "R19" H 3070 2296 50  0000 L CNN
F 1 "10k" H 3070 2205 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2930 2250 50  0001 C CNN
F 3 "" H 3000 2250 50  0001 C CNN
	1    3000 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 2100 3000 2050
Connection ~ 3000 2050
Wire Wire Line
	3000 2050 2850 2050
Wire Wire Line
	3000 2400 3000 2600
Text Label 2950 1650 0    50   ~ 0
COM_SRC
$EndSCHEMATC
