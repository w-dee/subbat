EESchema Schematic File Version 4
LIBS:subbat-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 16
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
L DeeComponents:USB_Ax2 J9
U 1 1 5A653EB4
P 5150 2400
AR Path="/5A653DCD/5A653EB4" Ref="J9"  Part="1" 
AR Path="/5A676C71/5A653EB4" Ref="J11"  Part="1" 
AR Path="/5A678AED/5A653EB4" Ref="J12"  Part="1" 
AR Path="/5A678AF2/5A653EB4" Ref="J13"  Part="1" 
F 0 "J13" H 5281 2867 50  0000 C CNN
F 1 "USB_Ax2" H 5281 2776 50  0000 C CNN
F 2 "components:USB_Ax2" H 5650 2250 50  0001 C CNN
F 3 "" H 5650 2250 50  0001 C CNN
	1    5150 2400
	-1   0    0    -1  
$EndComp
$Comp
L device:D_Zener_Small D15
U 1 1 5A659FB9
P 4350 2300
AR Path="/5A653DCD/5A659FB9" Ref="D15"  Part="1" 
AR Path="/5A676C71/5A659FB9" Ref="D17"  Part="1" 
AR Path="/5A678AED/5A659FB9" Ref="D19"  Part="1" 
AR Path="/5A678AF2/5A659FB9" Ref="D21"  Part="1" 
F 0 "D21" V 4304 2368 50  0000 L CNN
F 1 "6V" V 4395 2368 50  0000 L CNN
F 2 "Diodes_SMD:D_SMA_Handsoldering" V 4350 2300 50  0001 C CNN
F 3 "https://en.wikipedia.org/wiki/Zener_diode" V 4350 2300 50  0001 C CNN
	1    4350 2300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR032
U 1 1 5A659FED
P 4350 3050
AR Path="/5A653DCD/5A659FED" Ref="#PWR032"  Part="1" 
AR Path="/5A676C71/5A659FED" Ref="#PWR041"  Part="1" 
AR Path="/5A678AED/5A659FED" Ref="#PWR045"  Part="1" 
AR Path="/5A678AF2/5A659FED" Ref="#PWR049"  Part="1" 
F 0 "#PWR049" H 4350 2800 50  0001 C CNN
F 1 "GND" H 4355 2877 50  0000 C CNN
F 2 "" H 4350 3050 50  0001 C CNN
F 3 "" H 4350 3050 50  0001 C CNN
	1    4350 3050
	1    0    0    -1  
$EndComp
$Comp
L device:D_Zener_Small D16
U 1 1 5A659FF3
P 4350 2750
AR Path="/5A653DCD/5A659FF3" Ref="D16"  Part="1" 
AR Path="/5A676C71/5A659FF3" Ref="D18"  Part="1" 
AR Path="/5A678AED/5A659FF3" Ref="D20"  Part="1" 
AR Path="/5A678AF2/5A659FF3" Ref="D22"  Part="1" 
F 0 "D22" V 4304 2818 50  0000 L CNN
F 1 "6V" V 4395 2818 50  0000 L CNN
F 2 "Diodes_SMD:D_SMA_Handsoldering" V 4350 2750 50  0001 C CNN
F 3 "https://en.wikipedia.org/wiki/Zener_diode" V 4350 2750 50  0001 C CNN
	1    4350 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	4700 2300 4650 2300
Wire Wire Line
	4650 2300 4650 2400
Wire Wire Line
	4650 2400 4700 2400
Wire Wire Line
	4700 2750 4650 2750
Wire Wire Line
	4650 2750 4650 2850
Wire Wire Line
	4650 2850 4700 2850
Wire Wire Line
	4700 2200 4350 2200
Wire Wire Line
	4700 2650 4350 2650
Wire Wire Line
	4700 2500 4550 2500
Wire Wire Line
	4350 2500 4350 2400
Wire Wire Line
	4350 2850 4350 2950
Wire Wire Line
	4700 2950 4550 2950
Connection ~ 4350 2950
Wire Wire Line
	4350 2950 4350 3050
$Comp
L power:GND #PWR033
U 1 1 5A65A539
P 5250 3100
AR Path="/5A653DCD/5A65A539" Ref="#PWR033"  Part="1" 
AR Path="/5A676C71/5A65A539" Ref="#PWR042"  Part="1" 
AR Path="/5A678AED/5A65A539" Ref="#PWR046"  Part="1" 
AR Path="/5A678AF2/5A65A539" Ref="#PWR050"  Part="1" 
F 0 "#PWR050" H 5250 2850 50  0001 C CNN
F 1 "GND" H 5255 2927 50  0000 C CNN
F 2 "" H 5250 3100 50  0001 C CNN
F 3 "" H 5250 3100 50  0001 C CNN
	1    5250 3100
	1    0    0    -1  
$EndComp
$Comp
L device:Polyfuse_Small F1
U 1 1 5A65A70E
P 3450 2150
AR Path="/5A653DCD/5A65A70E" Ref="F1"  Part="1" 
AR Path="/5A676C71/5A65A70E" Ref="F5"  Part="1" 
AR Path="/5A678AED/5A65A70E" Ref="F9"  Part="1" 
AR Path="/5A678AF2/5A65A70E" Ref="F13"  Part="1" 
F 0 "F13" V 3245 2150 50  0000 C CNN
F 1 "2A" V 3336 2150 50  0000 C CNN
F 2 "Fuse_Holders_and_Fuses:Fuse_SMD1206_HandSoldering" H 3500 1950 50  0001 L CNN
F 3 "" H 3450 2150 50  0001 C CNN
	1    3450 2150
	0    1    1    0   
$EndComp
$Comp
L device:Polyfuse_Small F2
U 1 1 5A65AA44
P 3450 2450
AR Path="/5A653DCD/5A65AA44" Ref="F2"  Part="1" 
AR Path="/5A676C71/5A65AA44" Ref="F6"  Part="1" 
AR Path="/5A678AED/5A65AA44" Ref="F10"  Part="1" 
AR Path="/5A678AF2/5A65AA44" Ref="F14"  Part="1" 
F 0 "F14" V 3245 2450 50  0000 C CNN
F 1 "2A" V 3336 2450 50  0000 C CNN
F 2 "Fuse_Holders_and_Fuses:Fuse_SMD1206_HandSoldering" H 3500 2250 50  0001 L CNN
F 3 "" H 3450 2450 50  0001 C CNN
	1    3450 2450
	0    1    1    0   
$EndComp
$Comp
L device:Polyfuse_Small F3
U 1 1 5A65ABC6
P 3450 2750
AR Path="/5A653DCD/5A65ABC6" Ref="F3"  Part="1" 
AR Path="/5A676C71/5A65ABC6" Ref="F7"  Part="1" 
AR Path="/5A678AED/5A65ABC6" Ref="F11"  Part="1" 
AR Path="/5A678AF2/5A65ABC6" Ref="F15"  Part="1" 
F 0 "F15" V 3245 2750 50  0000 C CNN
F 1 "2A" V 3336 2750 50  0000 C CNN
F 2 "Fuse_Holders_and_Fuses:Fuse_SMD1206_HandSoldering" H 3500 2550 50  0001 L CNN
F 3 "" H 3450 2750 50  0001 C CNN
	1    3450 2750
	0    1    1    0   
$EndComp
$Comp
L device:Polyfuse_Small F4
U 1 1 5A65ABCC
P 3450 3050
AR Path="/5A653DCD/5A65ABCC" Ref="F4"  Part="1" 
AR Path="/5A676C71/5A65ABCC" Ref="F8"  Part="1" 
AR Path="/5A678AED/5A65ABCC" Ref="F12"  Part="1" 
AR Path="/5A678AF2/5A65ABCC" Ref="F16"  Part="1" 
F 0 "F16" V 3245 3050 50  0000 C CNN
F 1 "2A" V 3336 3050 50  0000 C CNN
F 2 "Fuse_Holders_and_Fuses:Fuse_SMD1206_HandSoldering" H 3500 2850 50  0001 L CNN
F 3 "" H 3450 3050 50  0001 C CNN
	1    3450 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	3550 2150 3550 2200
Wire Wire Line
	3550 2750 3550 3050
Connection ~ 4350 2200
Connection ~ 3550 2200
Wire Wire Line
	3550 2200 3550 2450
Wire Wire Line
	4350 2650 4100 2650
Wire Wire Line
	3550 2650 3550 2750
Connection ~ 4350 2650
Connection ~ 3550 2750
Wire Wire Line
	3350 2150 3200 2150
Wire Wire Line
	3200 2150 3200 2450
Wire Wire Line
	3200 3050 3350 3050
Wire Wire Line
	3350 2750 3200 2750
Connection ~ 3200 2750
Wire Wire Line
	3200 2750 3200 3050
Wire Wire Line
	3350 2450 3200 2450
Connection ~ 3200 2450
Wire Wire Line
	3200 2450 3200 2600
Text HLabel 2600 2600 0    50   Input ~ 0
VBUS_COM
Wire Wire Line
	2600 2600 3200 2600
Connection ~ 3200 2600
Wire Wire Line
	3200 2600 3200 2750
Wire Wire Line
	4550 2500 4550 2950
Connection ~ 4550 2500
Wire Wire Line
	4550 2500 4350 2500
Connection ~ 4550 2950
Wire Wire Line
	4550 2950 4350 2950
$Comp
L device:C_Small C8
U 1 1 5A67D167
P 4100 2300
AR Path="/5A653DCD/5A67D167" Ref="C8"  Part="1" 
AR Path="/5A676C71/5A67D167" Ref="C11"  Part="1" 
AR Path="/5A678AED/5A67D167" Ref="C13"  Part="1" 
AR Path="/5A678AF2/5A67D167" Ref="C15"  Part="1" 
F 0 "C15" H 4192 2346 50  0000 L CNN
F 1 "1u" H 4192 2255 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 4100 2300 50  0001 C CNN
F 3 "" H 4100 2300 50  0001 C CNN
	1    4100 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 5A67E493
P 4100 2400
AR Path="/5A653DCD/5A67E493" Ref="#PWR030"  Part="1" 
AR Path="/5A676C71/5A67E493" Ref="#PWR039"  Part="1" 
AR Path="/5A678AED/5A67E493" Ref="#PWR043"  Part="1" 
AR Path="/5A678AF2/5A67E493" Ref="#PWR047"  Part="1" 
F 0 "#PWR047" H 4100 2150 50  0001 C CNN
F 1 "GND" H 4105 2227 50  0000 C CNN
F 2 "" H 4100 2400 50  0001 C CNN
F 3 "" H 4100 2400 50  0001 C CNN
	1    4100 2400
	1    0    0    -1  
$EndComp
$Comp
L device:C_Small C9
U 1 1 5A67E9A5
P 4100 2750
AR Path="/5A653DCD/5A67E9A5" Ref="C9"  Part="1" 
AR Path="/5A676C71/5A67E9A5" Ref="C12"  Part="1" 
AR Path="/5A678AED/5A67E9A5" Ref="C14"  Part="1" 
AR Path="/5A678AF2/5A67E9A5" Ref="C16"  Part="1" 
F 0 "C16" H 4192 2796 50  0000 L CNN
F 1 "1u" H 4192 2705 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 4100 2750 50  0001 C CNN
F 3 "" H 4100 2750 50  0001 C CNN
	1    4100 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 5A67E9AB
P 4100 2850
AR Path="/5A653DCD/5A67E9AB" Ref="#PWR031"  Part="1" 
AR Path="/5A676C71/5A67E9AB" Ref="#PWR040"  Part="1" 
AR Path="/5A678AED/5A67E9AB" Ref="#PWR044"  Part="1" 
AR Path="/5A678AF2/5A67E9AB" Ref="#PWR048"  Part="1" 
F 0 "#PWR048" H 4100 2600 50  0001 C CNN
F 1 "GND" H 4105 2677 50  0000 C CNN
F 2 "" H 4100 2850 50  0001 C CNN
F 3 "" H 4100 2850 50  0001 C CNN
	1    4100 2850
	1    0    0    -1  
$EndComp
Connection ~ 4100 2650
Wire Wire Line
	4100 2650 3550 2650
Connection ~ 4100 2200
Wire Wire Line
	4100 2200 3550 2200
Wire Wire Line
	4100 2200 4350 2200
$EndSCHEMATC
