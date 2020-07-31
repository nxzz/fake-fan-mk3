EESchema Schematic File Version 4
LIBS:fake-fan-mk2-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
NoConn ~ 4800 950 
Wire Wire Line
	4800 1050 4950 1050
Wire Wire Line
	4800 1150 4950 1150
Text Label 3950 1750 2    50   ~ 0
FAN1
$Comp
L power:GND #PWR016
U 1 1 5C991D41
P 9750 5000
F 0 "#PWR016" H 9750 4750 50  0001 C CNN
F 1 "GND" H 9755 4827 50  0000 C CNN
F 2 "" H 9750 5000 50  0001 C CNN
F 3 "" H 9750 5000 50  0001 C CNN
	1    9750 5000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR08
U 1 1 5C9925AB
P 9750 2650
F 0 "#PWR08" H 9750 2500 50  0001 C CNN
F 1 "+3.3V" H 9765 2823 50  0000 C CNN
F 2 "" H 9750 2650 50  0001 C CNN
F 3 "" H 9750 2650 50  0001 C CNN
	1    9750 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 2650 9750 2800
Wire Wire Line
	9750 2800 9850 2800
Wire Wire Line
	9850 2800 9850 2900
Connection ~ 9750 2800
Wire Wire Line
	9750 2800 9750 2900
$Comp
L power:GND #PWR013
U 1 1 5C9A000D
P 8900 3300
F 0 "#PWR013" H 8900 3050 50  0001 C CNN
F 1 "GND" V 8905 3172 50  0000 R CNN
F 2 "" H 8900 3300 50  0001 C CNN
F 3 "" H 8900 3300 50  0001 C CNN
	1    8900 3300
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R1
U 1 1 5C9A00B2
P 9050 3300
F 0 "R1" V 8950 3300 50  0000 C CNN
F 1 "10k" V 9150 3300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 9050 3300 50  0001 C CNN
F 3 "~" H 9050 3300 50  0001 C CNN
	1    9050 3300
	0    1    -1   0   
$EndComp
Wire Wire Line
	8900 3300 8950 3300
$Comp
L power:GND #PWR07
U 1 1 5C9A7C83
P 7800 2650
F 0 "#PWR07" H 7800 2400 50  0001 C CNN
F 1 "GND" V 7805 2522 50  0000 R CNN
F 2 "" H 7800 2650 50  0001 C CNN
F 3 "" H 7800 2650 50  0001 C CNN
	1    7800 2650
	0    1    1    0   
$EndComp
Text Label 8950 3100 2    50   ~ 0
NRST
Text Label 8500 2750 0    50   ~ 0
NRST
Text Label 8500 2650 0    50   ~ 0
SWDIO
Text Label 7800 2750 2    50   ~ 0
SYS_CLK
Wire Wire Line
	8950 3100 9250 3100
Text Label 10450 4500 0    50   ~ 0
SYS_CLK
Text Label 10450 4400 0    50   ~ 0
SWDIO
Wire Wire Line
	10450 4400 10350 4400
Wire Wire Line
	10350 4500 10450 4500
$Comp
L Device:C_Small C2
U 1 1 5C9BE6B4
P 10150 5850
F 0 "C2" H 10050 5800 50  0000 R CNN
F 1 "0.1u" H 10050 5900 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10150 5850 50  0001 C CNN
F 3 "~" H 10150 5850 50  0001 C CNN
	1    10150 5850
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5C9BE818
P 10500 5850
F 0 "C3" H 10408 5804 50  0000 R CNN
F 1 "0.1u" H 10408 5895 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10500 5850 50  0001 C CNN
F 3 "~" H 10500 5850 50  0001 C CNN
	1    10500 5850
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR018
U 1 1 5C9BE866
P 10500 6100
F 0 "#PWR018" H 10500 5850 50  0001 C CNN
F 1 "GND" H 10505 5927 50  0000 C CNN
F 2 "" H 10500 6100 50  0001 C CNN
F 3 "" H 10500 6100 50  0001 C CNN
	1    10500 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 5950 10150 6000
Wire Wire Line
	10150 6000 10500 6000
Wire Wire Line
	10500 6000 10500 5950
$Comp
L power:+3.3V #PWR017
U 1 1 5C9C1D8E
P 10500 5600
F 0 "#PWR017" H 10500 5450 50  0001 C CNN
F 1 "+3.3V" H 10515 5773 50  0000 C CNN
F 2 "" H 10500 5600 50  0001 C CNN
F 3 "" H 10500 5600 50  0001 C CNN
	1    10500 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 5700 10500 5700
Wire Wire Line
	10500 5700 10500 5750
Wire Wire Line
	10150 5700 10150 5750
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 5C9D3692
P 4600 1050
F 0 "J2" H 4600 750 50  0000 C CNN
F 1 "FOUT1" H 4600 1250 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4600 1050 50  0001 C CNN
F 3 "~" H 4600 1050 50  0001 C CNN
	1    4600 1050
	-1   0    0    1   
$EndComp
Wire Wire Line
	4800 850  4900 850 
Wire Wire Line
	4900 850  4900 650 
$Comp
L power:GND #PWR05
U 1 1 5C9E48C8
P 4950 1450
F 0 "#PWR05" H 4950 1200 50  0001 C CNN
F 1 "GND" V 4955 1322 50  0000 R CNN
F 2 "" H 4950 1450 50  0001 C CNN
F 3 "" H 4950 1450 50  0001 C CNN
	1    4950 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 650  4900 650 
Wire Wire Line
	4300 650  4300 1750
$Comp
L Device:R_POT RV1
U 1 1 5CA931F2
P 3950 3600
F 0 "RV1" H 3880 3646 50  0000 R CNN
F 1 "22k" H 3880 3555 50  0000 R CNN
F 2 "pot:PVZ3A" H 3950 3600 50  0001 C CNN
F 3 "~" H 3950 3600 50  0001 C CNN
	1    3950 3600
	1    0    0    -1  
$EndComp
$Comp
L MCU_ST_STM32F0:STM32F030K6Tx U2
U 1 1 5CAA87C4
P 9850 3800
F 0 "U2" H 10300 5000 50  0000 C CNN
F 1 "STM32F030K6Tx" H 10300 4900 50  0000 C CNN
F 2 "Package_QFP:LQFP-32_7x7mm_P0.8mm" H 9350 2900 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00088500.pdf" H 9850 3800 50  0001 C CNN
	1    9850 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 3300 9250 3300
Wire Wire Line
	9750 4800 9750 4900
Wire Wire Line
	9850 4800 9850 4900
Wire Wire Line
	9850 4900 9750 4900
Connection ~ 9750 4900
Wire Wire Line
	9750 4900 9750 5000
Wire Wire Line
	9950 2900 9950 2800
Wire Wire Line
	9950 2800 9850 2800
Connection ~ 9850 2800
$Comp
L Device:C_Small C4
U 1 1 5CAE3D3C
P 10850 5850
F 0 "C4" H 10758 5804 50  0000 R CNN
F 1 "0.1u" H 10758 5895 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10850 5850 50  0001 C CNN
F 3 "~" H 10850 5850 50  0001 C CNN
	1    10850 5850
	-1   0    0    1   
$EndComp
Wire Wire Line
	10500 6000 10850 6000
Wire Wire Line
	10850 6000 10850 5950
Connection ~ 10500 6000
Wire Wire Line
	10500 5700 10850 5700
Wire Wire Line
	10850 5700 10850 5750
Connection ~ 10500 5700
Text Label 10550 3100 0    50   ~ 0
VR1
Wire Wire Line
	10350 3100 10550 3100
Text Label 10550 4200 0    50   ~ 0
FAN1
Text Label 9150 4300 2    50   ~ 0
FAKE1
Text Label 4150 3600 0    50   ~ 0
VR1
$Comp
L power:GND #PWR015
U 1 1 5CB3DF6B
P 3950 3850
F 0 "#PWR015" H 3950 3600 50  0001 C CNN
F 1 "GND" H 3955 3677 50  0000 C CNN
F 2 "" H 3950 3850 50  0001 C CNN
F 3 "" H 3950 3850 50  0001 C CNN
	1    3950 3850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR014
U 1 1 5CB3DFE2
P 3950 3350
F 0 "#PWR014" H 3950 3200 50  0001 C CNN
F 1 "+3.3V" H 3965 3523 50  0000 C CNN
F 2 "" H 3950 3350 50  0001 C CNN
F 3 "" H 3950 3350 50  0001 C CNN
	1    3950 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 3600 4150 3600
Wire Wire Line
	10350 4200 10550 4200
Wire Wire Line
	9150 4300 9250 4300
Wire Wire Line
	10500 5600 10500 5700
Wire Wire Line
	10500 6000 10500 6100
NoConn ~ 10350 4600
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5CE2743E
P 3800 1050
F 0 "J1" H 3800 750 50  0000 C CNN
F 1 "FIN1" H 3800 1250 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3800 1050 50  0001 C CNN
F 3 "~" H 3800 1050 50  0001 C CNN
	1    3800 1050
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5CE274DD
P 4150 1300
F 0 "#PWR04" H 4150 1050 50  0001 C CNN
F 1 "GND" H 4155 1127 50  0000 C CNN
F 2 "" H 4150 1300 50  0001 C CNN
F 3 "" H 4150 1300 50  0001 C CNN
	1    4150 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 1050 4250 1050
Wire Wire Line
	4250 1050 4250 600 
Wire Wire Line
	4250 600  4950 600 
Wire Wire Line
	4000 1150 4150 1150
Wire Wire Line
	4150 1150 4150 1300
Text Label 4150 850  1    50   ~ 0
FAKE1
NoConn ~ 4000 850 
Wire Wire Line
	4000 950  4150 950 
Wire Wire Line
	4150 950  4150 850 
Wire Notes Line
	3650 550  3650 2250
Wire Notes Line
	3650 2250 5450 2250
Wire Notes Line
	5450 2250 5450 550 
Wire Notes Line
	5450 550  3650 550 
Text Notes 3700 2200 0    50   ~ 0
FAN1
$Comp
L power:+12V #PWR01
U 1 1 5D43791D
P 3200 700
F 0 "#PWR01" H 3200 550 50  0001 C CNN
F 1 "+12V" V 3215 828 50  0000 L CNN
F 2 "" H 3200 700 50  0001 C CNN
F 3 "" H 3200 700 50  0001 C CNN
	1    3200 700 
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5D4E0AE9
P 1450 1750
F 0 "#PWR06" H 1450 1500 50  0001 C CNN
F 1 "GND" V 1455 1622 50  0000 R CNN
F 2 "" H 1450 1750 50  0001 C CNN
F 3 "" H 1450 1750 50  0001 C CNN
	1    1450 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR02
U 1 1 5D51B355
P 700 1050
F 0 "#PWR02" H 700 900 50  0001 C CNN
F 1 "+12V" H 715 1223 50  0000 C CNN
F 2 "" H 700 1050 50  0001 C CNN
F 3 "" H 700 1050 50  0001 C CNN
	1    700  1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	700  1200 950  1200
$Comp
L power:+3.3V #PWR03
U 1 1 5DB0AAE2
P 2350 1050
F 0 "#PWR03" H 2350 900 50  0001 C CNN
F 1 "+3.3V" H 2365 1223 50  0000 C CNN
F 2 "" H 2350 1050 50  0001 C CNN
F 3 "" H 2350 1050 50  0001 C CNN
	1    2350 1050
	1    0    0    -1  
$EndComp
Wire Notes Line
	550  550  2800 550 
Wire Notes Line
	2800 550  2800 2250
Wire Notes Line
	2800 2250 550  2250
Wire Notes Line
	550  2250 550  550 
Text Notes 600  2200 0    50   ~ 0
DCDC
Wire Wire Line
	3600 700  3600 600 
Wire Wire Line
	3600 600  4250 600 
Connection ~ 4250 600 
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5DD87977
P 2200 2600
F 0 "#FLG01" H 2200 2675 50  0001 C CNN
F 1 "PWR_FLAG" H 2200 2774 50  0000 C CNN
F 2 "" H 2200 2600 50  0001 C CNN
F 3 "~" H 2200 2600 50  0001 C CNN
	1    2200 2600
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR09
U 1 1 5DD87CD9
P 2200 2700
F 0 "#PWR09" H 2200 2550 50  0001 C CNN
F 1 "+12V" H 2215 2873 50  0000 C CNN
F 2 "" H 2200 2700 50  0001 C CNN
F 3 "" H 2200 2700 50  0001 C CNN
	1    2200 2700
	-1   0    0    1   
$EndComp
Wire Wire Line
	2200 2600 2200 2700
Wire Notes Line
	3650 3050 6150 3050
Wire Notes Line
	6150 3050 6150 5100
Wire Notes Line
	6150 5100 3650 5100
Wire Notes Line
	3650 5100 3650 3050
Text Notes 5900 3150 0    50   ~ 0
POT
Wire Notes Line
	7450 2400 11150 2400
Wire Notes Line
	11150 2400 11150 5250
Wire Notes Line
	11150 5250 7450 5250
Wire Notes Line
	7450 5250 7450 2400
Text Notes 7500 5200 0    50   ~ 0
MCU
NoConn ~ 9250 4500
NoConn ~ 9250 4600
$Comp
L Connector_Generic:Conn_02x02_Counter_Clockwise J3
U 1 1 5CCA1F41
P 8100 2650
F 0 "J3" H 8150 2867 50  0000 C CNN
F 1 "SWD" H 8150 2776 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical" H 8100 2650 50  0001 C CNN
F 3 "~" H 8100 2650 50  0001 C CNN
	1    8100 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 2750 7900 2750
Wire Wire Line
	7800 2650 7900 2650
Wire Wire Line
	8400 2650 8500 2650
Wire Wire Line
	8500 2750 8400 2750
Wire Wire Line
	3950 1750 4300 1750
Wire Wire Line
	4950 1150 4950 1450
Wire Wire Line
	4950 600  4950 1050
Wire Wire Line
	3200 700  3600 700 
Wire Wire Line
	3950 3750 3950 3850
Wire Wire Line
	3950 3350 3950 3450
NoConn ~ 9250 3800
NoConn ~ 9250 3700
NoConn ~ 10350 3200
NoConn ~ 10350 3300
NoConn ~ 10350 3400
NoConn ~ 10350 3500
NoConn ~ 10350 3600
NoConn ~ 10350 3700
NoConn ~ 10350 3800
NoConn ~ 10350 3900
NoConn ~ 10350 4000
NoConn ~ 10350 4100
NoConn ~ 10350 4300
NoConn ~ 9250 4000
NoConn ~ 9250 4100
NoConn ~ 9250 4200
NoConn ~ 9250 4400
$Comp
L akipart:NJM12888 U1
U 1 1 5F16913F
P 1450 1300
F 0 "U1" H 1450 1625 50  0000 C CNN
F 1 "NJM12888" H 1450 1534 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 1450 1300 50  0001 C CNN
F 3 "" H 1450 1300 50  0001 C CNN
	1    1450 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	700  1050 700  1200
Wire Wire Line
	1450 1550 1450 1650
Wire Wire Line
	1100 1400 950  1400
Wire Wire Line
	950  1400 950  1200
Connection ~ 950  1200
Wire Wire Line
	950  1200 1100 1200
NoConn ~ 1800 1400
Wire Wire Line
	1800 1200 2350 1200
Wire Wire Line
	2350 1200 2350 1050
$Comp
L Device:C_Small C1
U 1 1 5F17564E
P 2350 1400
F 0 "C1" H 2250 1350 50  0000 R CNN
F 1 "0.47u" H 2250 1450 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2350 1400 50  0001 C CNN
F 3 "~" H 2350 1400 50  0001 C CNN
	1    2350 1400
	-1   0    0    1   
$EndComp
Wire Wire Line
	1450 1650 2350 1650
Wire Wire Line
	2350 1650 2350 1500
Connection ~ 1450 1650
Wire Wire Line
	1450 1650 1450 1750
Wire Wire Line
	2350 1300 2350 1200
Connection ~ 2350 1200
$Comp
L Device:C_Small C?
U 1 1 5F17FBA4
P 700 1400
F 0 "C?" H 608 1354 50  0000 R CNN
F 1 "0.1u" H 608 1445 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 700 1400 50  0001 C CNN
F 3 "~" H 700 1400 50  0001 C CNN
	1    700  1400
	-1   0    0    1   
$EndComp
Wire Wire Line
	700  1200 700  1300
Connection ~ 700  1200
Wire Wire Line
	700  1500 700  1650
Wire Wire Line
	700  1650 1450 1650
$EndSCHEMATC
