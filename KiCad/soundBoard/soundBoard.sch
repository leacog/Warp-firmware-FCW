EESchema Schematic File Version 4
EELAYER 30 0
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
L 01_soundBoard:AG_ElectretMic U2
U 1 1 61DBF0B4
P 3600 1900
F 0 "U2" V 4065 1967 50  0000 C CNN
F 1 "AG_ElectretMic" V 3974 1967 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 3550 1650 50  0001 C CNN
F 3 "" H 3550 1650 50  0001 C CNN
	1    3600 1900
	0    -1   -1   0   
$EndComp
$Comp
L 01_soundBoard:FRDM-KL03Z U1
U 1 1 61DBF33D
P 5550 2350
F 0 "U1" H 5750 1185 50  0000 C CNN
F 1 "FRDM-KL03Z" H 5750 1276 50  0000 C CNN
F 2 "Module:Arduino_UNO_R3" H 6200 3450 50  0001 C CNN
F 3 "" H 6200 3450 50  0001 C CNN
	1    5550 2350
	-1   0    0    1   
$EndComp
$Comp
L 01_soundBoard:RealAG_ElectretMic U3
U 1 1 61DBFE7C
P 3750 2950
F 0 "U3" V 4315 3167 50  0000 C CNN
F 1 "RealAG_ElectretMic" V 4224 3167 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 3750 2950 50  0001 C CNN
F 3 "" H 3750 2950 50  0001 C CNN
	1    3750 2950
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_BJT:BCP56 Q3
U 1 1 61DC0A5E
P 7300 4550
F 0 "Q3" H 7491 4504 50  0000 L CNN
F 1 "BCP56" H 7491 4595 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 7500 4475 50  0001 L CIN
F 3 "http://cache.nxp.com/documents/data_sheet/BCP56_BCX56_BC56PA.pdf?pspll=1" H 7300 4550 50  0001 L CNN
	1    7300 4550
	-1   0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 61DC25B4
P 7600 4300
F 0 "R3" H 7670 4346 50  0000 L CNN
F 1 "670" H 7670 4255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7530 4300 50  0001 C CNN
F 3 "~" H 7600 4300 50  0001 C CNN
	1    7600 4300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Barrel_Jack_MountingPin J4
U 1 1 61DC2D05
P 7700 1550
F 0 "J4" H 7470 1422 50  0000 R CNN
F 1 "12_Barrel_Jack" H 7470 1513 50  0000 R CNN
F 2 "Connector_BarrelJack:BarrelJack_Horizontal" H 7750 1510 50  0001 C CNN
F 3 "~" H 7750 1510 50  0001 C CNN
	1    7700 1550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 61DC3E27
P 8000 1650
F 0 "#PWR0101" H 8000 1400 50  0001 C CNN
F 1 "GND" V 8005 1522 50  0000 R CNN
F 2 "" H 8000 1650 50  0001 C CNN
F 3 "" H 8000 1650 50  0001 C CNN
	1    8000 1650
	0    -1   -1   0   
$EndComp
$Comp
L power:+12V #PWR0102
U 1 1 61DC42AA
P 8000 1450
F 0 "#PWR0102" H 8000 1300 50  0001 C CNN
F 1 "+12V" V 8015 1578 50  0000 L CNN
F 2 "" H 8000 1450 50  0001 C CNN
F 3 "" H 8000 1450 50  0001 C CNN
	1    8000 1450
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x02_Male J3
U 1 1 61DC5806
P 7200 5200
F 0 "J3" V 7354 5012 50  0000 R CNN
F 1 "GREEN" V 7263 5012 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 7200 5200 50  0001 C CNN
F 3 "~" H 7200 5200 50  0001 C CNN
	1    7200 5200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7600 4450 7600 4550
Wire Wire Line
	7600 4550 7500 4550
Wire Wire Line
	7200 4750 7200 5000
Wire Wire Line
	7300 5000 7300 4850
Wire Wire Line
	7300 4850 7400 4850
$Comp
L power:+12V #PWR0103
U 1 1 61DD9394
P 7400 4850
F 0 "#PWR0103" H 7400 4700 50  0001 C CNN
F 1 "+12V" V 7415 4978 50  0000 L CNN
F 2 "" H 7400 4850 50  0001 C CNN
F 3 "" H 7400 4850 50  0001 C CNN
	1    7400 4850
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 61DD9BEE
P 7200 4350
F 0 "#PWR0104" H 7200 4100 50  0001 C CNN
F 1 "GND" H 7205 4177 50  0000 C CNN
F 2 "" H 7200 4350 50  0001 C CNN
F 3 "" H 7200 4350 50  0001 C CNN
	1    7200 4350
	-1   0    0    1   
$EndComp
$Comp
L Transistor_BJT:BCP56 Q2
U 1 1 61DDDA49
P 6200 4550
F 0 "Q2" H 6391 4504 50  0000 L CNN
F 1 "BCP56" H 6391 4595 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 6400 4475 50  0001 L CIN
F 3 "http://cache.nxp.com/documents/data_sheet/BCP56_BCX56_BC56PA.pdf?pspll=1" H 6200 4550 50  0001 L CNN
	1    6200 4550
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 61DDDA4F
P 6500 4300
F 0 "R2" H 6570 4346 50  0000 L CNN
F 1 "670" H 6570 4255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6430 4300 50  0001 C CNN
F 3 "~" H 6500 4300 50  0001 C CNN
	1    6500 4300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J2
U 1 1 61DDDA55
P 6100 5200
F 0 "J2" V 6254 5012 50  0000 R CNN
F 1 "BLUE" V 6163 5012 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 6100 5200 50  0001 C CNN
F 3 "~" H 6100 5200 50  0001 C CNN
	1    6100 5200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6500 4450 6500 4550
Wire Wire Line
	6500 4550 6400 4550
Wire Wire Line
	6100 4750 6100 5000
Wire Wire Line
	6200 5000 6200 4850
Wire Wire Line
	6200 4850 6300 4850
$Comp
L power:+12V #PWR0105
U 1 1 61DDDA60
P 6300 4850
F 0 "#PWR0105" H 6300 4700 50  0001 C CNN
F 1 "+12V" V 6315 4978 50  0000 L CNN
F 2 "" H 6300 4850 50  0001 C CNN
F 3 "" H 6300 4850 50  0001 C CNN
	1    6300 4850
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 61DDDA66
P 6100 4350
F 0 "#PWR0106" H 6100 4100 50  0001 C CNN
F 1 "GND" H 6105 4177 50  0000 C CNN
F 2 "" H 6100 4350 50  0001 C CNN
F 3 "" H 6100 4350 50  0001 C CNN
	1    6100 4350
	-1   0    0    1   
$EndComp
$Comp
L Transistor_BJT:BCP56 Q1
U 1 1 61DDF79D
P 5150 4550
F 0 "Q1" H 5341 4504 50  0000 L CNN
F 1 "BCP56" H 5341 4595 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 5350 4475 50  0001 L CIN
F 3 "http://cache.nxp.com/documents/data_sheet/BCP56_BCX56_BC56PA.pdf?pspll=1" H 5150 4550 50  0001 L CNN
	1    5150 4550
	-1   0    0    1   
$EndComp
$Comp
L Device:R R1
U 1 1 61DDF7A3
P 5450 4300
F 0 "R1" H 5520 4346 50  0000 L CNN
F 1 "670" H 5520 4255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5380 4300 50  0001 C CNN
F 3 "~" H 5450 4300 50  0001 C CNN
	1    5450 4300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 61DDF7A9
P 5050 5200
F 0 "J1" V 5204 5012 50  0000 R CNN
F 1 "RED" V 5113 5012 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5050 5200 50  0001 C CNN
F 3 "~" H 5050 5200 50  0001 C CNN
	1    5050 5200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5450 4450 5450 4550
Wire Wire Line
	5450 4550 5350 4550
Wire Wire Line
	5050 4750 5050 5000
Wire Wire Line
	5150 5000 5150 4850
Wire Wire Line
	5150 4850 5250 4850
$Comp
L power:+12V #PWR0107
U 1 1 61DDF7B4
P 5250 4850
F 0 "#PWR0107" H 5250 4700 50  0001 C CNN
F 1 "+12V" V 5265 4978 50  0000 L CNN
F 2 "" H 5250 4850 50  0001 C CNN
F 3 "" H 5250 4850 50  0001 C CNN
	1    5250 4850
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 61DDF7BA
P 5050 4350
F 0 "#PWR0108" H 5050 4100 50  0001 C CNN
F 1 "GND" H 5055 4177 50  0000 C CNN
F 2 "" H 5050 4350 50  0001 C CNN
F 3 "" H 5050 4350 50  0001 C CNN
	1    5050 4350
	-1   0    0    1   
$EndComp
NoConn ~ 7700 1850
$Comp
L power:VCC #PWR0109
U 1 1 61DE27AA
P 5900 2650
F 0 "#PWR0109" H 5900 2500 50  0001 C CNN
F 1 "VCC" V 5917 2778 50  0000 L CNN
F 2 "" H 5900 2650 50  0001 C CNN
F 3 "" H 5900 2650 50  0001 C CNN
	1    5900 2650
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR0110
U 1 1 61DE3444
P 3750 2800
F 0 "#PWR0110" H 3750 2650 50  0001 C CNN
F 1 "VCC" V 3767 2928 50  0000 L CNN
F 2 "" H 3750 2800 50  0001 C CNN
F 3 "" H 3750 2800 50  0001 C CNN
	1    3750 2800
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR0111
U 1 1 61DE49A2
P 3750 1750
F 0 "#PWR0111" H 3750 1600 50  0001 C CNN
F 1 "VCC" V 3767 1878 50  0000 L CNN
F 2 "" H 3750 1750 50  0001 C CNN
F 3 "" H 3750 1750 50  0001 C CNN
	1    3750 1750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 61DE5015
P 3750 2650
F 0 "#PWR0112" H 3750 2400 50  0001 C CNN
F 1 "GND" V 3755 2522 50  0000 R CNN
F 2 "" H 3750 2650 50  0001 C CNN
F 3 "" H 3750 2650 50  0001 C CNN
	1    3750 2650
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 61DE5A5E
P 3750 1900
F 0 "#PWR0113" H 3750 1650 50  0001 C CNN
F 1 "GND" V 3755 1772 50  0000 R CNN
F 2 "" H 3750 1900 50  0001 C CNN
F 3 "" H 3750 1900 50  0001 C CNN
	1    3750 1900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 61DE5E43
P 4800 3050
F 0 "#PWR0114" H 4800 2800 50  0001 C CNN
F 1 "GND" V 4805 2922 50  0000 R CNN
F 2 "" H 4800 3050 50  0001 C CNN
F 3 "" H 4800 3050 50  0001 C CNN
	1    4800 3050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 61DE66CE
P 5900 2350
F 0 "#PWR0115" H 5900 2100 50  0001 C CNN
F 1 "GND" V 5905 2222 50  0000 R CNN
F 2 "" H 5900 2350 50  0001 C CNN
F 3 "" H 5900 2350 50  0001 C CNN
	1    5900 2350
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 61DE6F7B
P 5900 2450
F 0 "#PWR0116" H 5900 2200 50  0001 C CNN
F 1 "GND" V 5905 2322 50  0000 R CNN
F 2 "" H 5900 2450 50  0001 C CNN
F 3 "" H 5900 2450 50  0001 C CNN
	1    5900 2450
	0    -1   -1   0   
$EndComp
NoConn ~ 5900 2550
NoConn ~ 5900 2250
NoConn ~ 5900 2950
NoConn ~ 5900 2850
$Comp
L Switch:SW_Push SW1
U 1 1 61DE7D8B
P 6450 2750
F 0 "SW1" H 6450 2950 50  0000 C CNN
F 1 "SW_Push" H 6450 3050 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 6450 2950 50  0001 C CNN
F 3 "~" H 6450 2950 50  0001 C CNN
	1    6450 2750
	-1   0    0    1   
$EndComp
Wire Wire Line
	5900 2750 6250 2750
Wire Wire Line
	6650 2750 7000 2750
Wire Wire Line
	7000 2750 7000 2850
$Comp
L power:GND #PWR0117
U 1 1 61DEA80B
P 7000 2850
F 0 "#PWR0117" H 7000 2600 50  0001 C CNN
F 1 "GND" H 7005 2677 50  0000 C CNN
F 2 "" H 7000 2850 50  0001 C CNN
F 3 "" H 7000 2850 50  0001 C CNN
	1    7000 2850
	1    0    0    -1  
$EndComp
Text Label 5450 4150 1    50   ~ 0
RED_PWM
Text Label 6500 4150 1    50   ~ 0
GREEN_PWM
Text Label 7600 4150 1    50   ~ 0
BLUE_PWM
Text Label 4650 1800 2    50   ~ 0
BLUE_PWM
Wire Wire Line
	4650 1800 4800 1800
Text Label 4650 2650 2    50   ~ 0
GREEN_PWM
Wire Wire Line
	4650 2650 4800 2650
Text Label 4650 2850 2    50   ~ 0
RED_PWM
Wire Wire Line
	4650 2850 4800 2850
Text Label 6100 2000 0    50   ~ 0
ADC_IN
Wire Wire Line
	5900 2000 6100 2000
Text Label 3850 3100 0    50   ~ 0
ADC_IN
Text Label 3850 2050 0    50   ~ 0
ADC_IN
Wire Wire Line
	3850 2050 3750 2050
Wire Wire Line
	3850 3100 3750 3100
Wire Wire Line
	3750 2950 4800 2950
Text Label 3850 3250 0    50   ~ 0
AR_ADJUST
Wire Wire Line
	3850 3250 3750 3250
Text Label 4650 1500 2    50   ~ 0
AR_ADJUST
Wire Wire Line
	4650 1500 4800 1500
NoConn ~ 4800 1600
NoConn ~ 4800 2450
NoConn ~ 4800 2550
NoConn ~ 4800 2750
NoConn ~ 4800 3150
NoConn ~ 4800 3250
NoConn ~ 4800 3350
NoConn ~ 5900 1500
NoConn ~ 5900 1600
NoConn ~ 5900 1700
NoConn ~ 5900 1800
NoConn ~ 5900 1900
$EndSCHEMATC