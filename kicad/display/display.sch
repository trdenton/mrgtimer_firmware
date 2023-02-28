EESchema Schematic File Version 4
LIBS:display-cache
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
$Comp
L Connector:Screw_Terminal_01x08 J1
U 1 1 5C77479C
P 1350 2650
F 0 "J1" H 1270 2025 50  0000 C CNN
F 1 "Screw_Terminal_01x08" H 1270 2116 50  0000 C CNN
F 2 "" H 1350 2650 50  0001 C CNN
F 3 "~" H 1350 2650 50  0001 C CNN
	1    1350 2650
	-1   0    0    -1  
$EndComp
$Comp
L MAX7219:MAX7219 U1
U 1 1 5C78A76D
P 3750 2100
F 0 "U1" H 3750 2926 50  0000 C CNN
F 1 "MAX7219" H 3750 2100 50  0001 L BNN
F 2 "Package_SO:SOIC-24W_7.5x15.4mm_P1.27mm" H 3750 2100 50  0001 C CNN
F 3 "None" H 3750 2100 50  0001 L BNN
F 4 "Maxim Integrated" H 3750 2100 50  0001 L BNN "Field4"
F 5 "None" H 3750 2100 50  0001 L BNN "Field5"
F 6 "Unavailable" H 3750 2100 50  0001 L BNN "Field7"
F 7 "MAX7219" H 3750 2100 50  0001 L BNN "Field8"
	1    3750 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5C79F81F
P 6050 1750
F 0 "R1" V 5843 1750 50  0000 C CNN
F 1 "R" V 5934 1750 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 5980 1750 50  0001 C CNN
F 3 "~" H 6050 1750 50  0001 C CNN
	1    6050 1750
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5C79F85E
P 6050 1850
F 0 "R2" V 5843 1850 50  0000 C CNN
F 1 "R" V 5934 1850 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 5980 1850 50  0001 C CNN
F 3 "~" H 6050 1850 50  0001 C CNN
	1    6050 1850
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5C79F8A8
P 6050 1950
F 0 "R3" V 5843 1950 50  0000 C CNN
F 1 "R" V 5934 1950 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 5980 1950 50  0001 C CNN
F 3 "~" H 6050 1950 50  0001 C CNN
	1    6050 1950
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5C79F8C8
P 6050 2050
F 0 "R4" V 5843 2050 50  0000 C CNN
F 1 "R" V 5934 2050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 5980 2050 50  0001 C CNN
F 3 "~" H 6050 2050 50  0001 C CNN
	1    6050 2050
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5C79F8F0
P 6050 2150
F 0 "R5" V 5843 2150 50  0000 C CNN
F 1 "R" V 5934 2150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 5980 2150 50  0001 C CNN
F 3 "~" H 6050 2150 50  0001 C CNN
	1    6050 2150
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 5C79F94A
P 6050 2250
F 0 "R6" V 5843 2250 50  0000 C CNN
F 1 "R" V 5934 2250 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 5980 2250 50  0001 C CNN
F 3 "~" H 6050 2250 50  0001 C CNN
	1    6050 2250
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 5C79F970
P 6050 2350
F 0 "R7" V 5843 2350 50  0000 C CNN
F 1 "R" V 5934 2350 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 5980 2350 50  0001 C CNN
F 3 "~" H 6050 2350 50  0001 C CNN
	1    6050 2350
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 5C79F998
P 6050 2450
F 0 "R8" V 5843 2450 50  0000 C CNN
F 1 "R" V 5934 2450 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 5980 2450 50  0001 C CNN
F 3 "~" H 6050 2450 50  0001 C CNN
	1    6050 2450
	0    1    1    0   
$EndComp
Wire Wire Line
	5700 1750 5900 1750
Wire Wire Line
	5700 1850 5900 1850
Wire Wire Line
	5700 1950 5900 1950
Wire Wire Line
	5700 2050 5900 2050
Wire Wire Line
	5700 2150 5900 2150
Wire Wire Line
	5700 2350 5900 2350
Wire Wire Line
	5900 2450 5700 2450
$Comp
L Transistor_Array:TBD62783A U2
U 1 1 5C78A3F1
P 5300 2050
F 0 "U2" H 5000 2650 50  0000 C CNN
F 1 "TBD62783A-NPN" H 5000 2550 50  0000 C CNN
F 2 "Package_SO:SOIC-18W_7.5x11.6mm_P1.27mm" H 5300 1500 50  0001 C CNN
F 3 "http://toshiba.semicon-storage.com/info/docget.jsp?did=30523&prodName=TBD62783APG" H 5000 2450 50  0001 C CNN
	1    5300 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 2250 5900 2250
Wire Wire Line
	7100 1750 6200 1750
Wire Wire Line
	6200 1850 7100 1850
Wire Wire Line
	7100 1950 6200 1950
Wire Wire Line
	6200 2050 7100 2050
Wire Wire Line
	7100 2150 6200 2150
Wire Wire Line
	6200 2250 7100 2250
Wire Wire Line
	7100 2350 6200 2350
Wire Wire Line
	6200 2450 7100 2450
$Comp
L power:+12V #PWR0101
U 1 1 5C7A066F
P 5300 1550
F 0 "#PWR0101" H 5300 1400 50  0001 C CNN
F 1 "+12V" H 5315 1723 50  0000 C CNN
F 2 "" H 5300 1550 50  0001 C CNN
F 3 "" H 5300 1550 50  0001 C CNN
	1    5300 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 1750 4900 1750
Wire Wire Line
	4300 1850 4900 1850
Wire Wire Line
	4300 1950 4900 1950
Wire Wire Line
	4900 2050 4300 2050
Wire Wire Line
	4300 2150 4900 2150
Wire Wire Line
	4900 2250 4300 2250
Wire Wire Line
	4300 2350 4900 2350
Wire Wire Line
	4300 1650 4600 1650
Wire Wire Line
	4600 1650 4600 2450
Wire Wire Line
	4600 2450 4900 2450
$Comp
L power:+5V #PWR0102
U 1 1 5C7A69EF
P 3200 2550
F 0 "#PWR0102" H 3200 2400 50  0001 C CNN
F 1 "+5V" V 3215 2678 50  0000 L CNN
F 2 "" H 3200 2550 50  0001 C CNN
F 3 "" H 3200 2550 50  0001 C CNN
	1    3200 2550
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5C7A6A9E
P 3200 2650
F 0 "#PWR0103" H 3200 2400 50  0001 C CNN
F 1 "GND" V 3205 2522 50  0000 R CNN
F 2 "" H 3200 2650 50  0001 C CNN
F 3 "" H 3200 2650 50  0001 C CNN
	1    3200 2650
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5C7A6B67
P 3200 2750
F 0 "#PWR0104" H 3200 2500 50  0001 C CNN
F 1 "GND" V 3205 2622 50  0000 R CNN
F 2 "" H 3200 2750 50  0001 C CNN
F 3 "" H 3200 2750 50  0001 C CNN
	1    3200 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	3200 1650 2750 1650
Wire Wire Line
	2750 1650 2750 3300
Wire Wire Line
	2750 3300 4900 3300
Wire Wire Line
	3200 1750 2650 1750
Wire Wire Line
	2650 1750 2650 3400
Wire Wire Line
	2650 3400 4900 3400
Wire Wire Line
	3200 1850 2550 1850
Wire Wire Line
	2550 1850 2550 3500
Wire Wire Line
	2550 3500 4900 3500
Wire Wire Line
	3200 1950 2450 1950
Wire Wire Line
	2450 1950 2450 3600
Wire Wire Line
	2450 3600 4900 3600
Wire Wire Line
	7700 3300 5700 3300
$Comp
L power:+12V #PWR0105
U 1 1 5C7C5D50
P 5300 3100
F 0 "#PWR0105" H 5300 2950 50  0001 C CNN
F 1 "+12V" H 5315 3273 50  0000 C CNN
F 2 "" H 5300 3100 50  0001 C CNN
F 3 "" H 5300 3100 50  0001 C CNN
	1    5300 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5C7C5DD0
P 5300 2650
F 0 "#PWR0106" H 5300 2400 50  0001 C CNN
F 1 "GND" H 5305 2477 50  0000 C CNN
F 2 "" H 5300 2650 50  0001 C CNN
F 3 "" H 5300 2650 50  0001 C CNN
	1    5300 2650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5C7C5DFC
P 5750 4750
F 0 "#PWR0107" H 5750 4500 50  0001 C CNN
F 1 "GND" H 5755 4577 50  0000 C CNN
F 2 "" H 5750 4750 50  0001 C CNN
F 3 "" H 5750 4750 50  0001 C CNN
	1    5750 4750
	0    1    1    0   
$EndComp
$Comp
L Display_Character:KCSA02-105 U4
U 1 1 5C7C6C1B
P 7400 2050
F 0 "U4" H 7400 2717 50  0000 C CNN
F 1 "KCSA02-105" H 7400 2626 50  0000 C CNN
F 2 "Display_7Segment:KCSC02-105" H 7400 1450 50  0001 C CNN
F 3 "http://www.kingbright.com/attachments/file/psearch/000/00/00/KCSA02-105(Ver.10A).pdf" H 6900 2525 50  0001 L CNN
	1    7400 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 2450 7700 3300
Wire Wire Line
	7700 2350 7700 2450
Connection ~ 7700 2450
$Comp
L 2019-03-04_00-13-58:SN75374D U3
U 1 1 5C7C8D83
P 5750 4050
F 0 "U3" H 6950 4437 60  0000 C CNN
F 1 "SN75374D" H 6950 4331 60  0000 C CNN
F 2 "Package_SO:SOIC-16W_5.3x10.2mm_P1.27mm" H 6950 4290 60  0001 C CNN
F 3 "" H 5750 4050 60  0000 C CNN
	1    5750 4050
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0108
U 1 1 5C7C901C
P 8150 4050
F 0 "#PWR0108" H 8150 3900 50  0001 C CNN
F 1 "+5V" V 8165 4178 50  0000 L CNN
F 2 "" H 8150 4050 50  0001 C CNN
F 3 "" H 8150 4050 50  0001 C CNN
	1    8150 4050
	0    1    1    0   
$EndComp
$Comp
L power:+12V #PWR0109
U 1 1 5C7C90C6
P 5750 4050
F 0 "#PWR0109" H 5750 3900 50  0001 C CNN
F 1 "+12V" V 5765 4178 50  0000 L CNN
F 2 "" H 5750 4050 50  0001 C CNN
F 3 "" H 5750 4050 50  0001 C CNN
	1    5750 4050
	0    -1   -1   0   
$EndComp
$Comp
L power:+12V #PWR0110
U 1 1 5C7C9D59
P 8150 4750
F 0 "#PWR0110" H 8150 4600 50  0001 C CNN
F 1 "+12V" V 8165 4878 50  0000 L CNN
F 2 "" H 8150 4750 50  0001 C CNN
F 3 "" H 8150 4750 50  0001 C CNN
	1    8150 4750
	0    1    1    0   
$EndComp
$EndSCHEMATC
