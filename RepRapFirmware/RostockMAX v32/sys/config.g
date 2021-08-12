; SeeMeCNC 3D Printers
; General preferences 
G90                                                     ; absolute coordinates
M83                                                     ; relative extruder moves

; Only remove ONE semi-colon for ONE printer configuration
M550 P"RMAXV32"                                         ; set printer name (ARTEMIS, RostockMAX, BOSSdelta, SeeMeCNC, BestFriend, etc.)
M665 R150 L340.5 B150 H400 X0 Y0 Z0                    ; INJECTION MOLDED ARMS delta radius, diagonal rod length, printable radius and homed height
;M665 R150 L350 B150 H350 X0 Y0 Z0                       ; CARBON FIBER ARMS length setting

M666 X0 Y0 Z0                                           ; endstop adjustment (this is set by autocalibration leveling)

; Network
M552 S1                                                 ; enable network
M586 P0 S1                                              ; enable HTTP
M586 P1 S0                                              ; disable FTP
M586 P2 S0                                              ; disable Telnet

; Drives
M569 P0 S0                                              ; physical drive 0
M569 P1 S0                                              ; physical drive 1
M569 P2 S0                                              ; physical drive 2
M569 P3 S0                                              ; physical drive 3   S one or zero = motor direction
M569 P4 S0                                              ; physical drive 4   S one or zero = motor direction
M584 X0 Y1 Z2 E3:4                                      ; set drive mapping
M350 X16 Y16 Z16 E16:16 I1                              ; configure micro stepping with interpolation
M92 X200.00 Y200.00 Z200.00 E182.00:182.00              ; 0.9 step/degree motors set steps per mm < adjust depending on pulley num. of Teeth
;M92 X100.00 Y100.00 Z100.00 E91.00:91.00                ; 16T PULLEYS 1.8 step/degree motors set steps per mm (older SeeMe printers <2017)
;M92 X80.00 Y80.00 Z80.00 E91.00:91.00                   ; 20T PULLEYS 1.8 step/degree motors set steps per mm (older SeeMe printers <2017)
M566 X700.00 Y700.00 Z700.00 E2000.00:2000.00           ; set maximum instantaneous speed changes (mm/min)
M203 X10000.00 Y10000.00 Z10000.00 E9000.00:9000.00     ; set maximum speeds (mm/min)
M201 X1400.00 Y1400.00 Z1400.00 E5000.00:5000.00        ; set accelerations (mm/s^2)
M906 X1500 Y1500 Z1500 E1300:1300 I40                   ; set motor currents (mA) and motor idle factor in per cent
M84 S30                                                 ; Set idle timeout

; Axis Limits
M208 Z0 S1                                              ; set minimum Z

; End-stops
M574 X2 S1 P"xstop"                                     ; configure active-high endstop for high end on X via pin xstop
M574 Y2 S1 P"ystop"                                     ; configure active-high endstop for high end on Y via pin ystop
M574 Z2 S1 P"zstop"                                     ; configure active-high endstop for high end on Z via pin zstop

; Z-Probe
M558 P5 I0 A4 R0.4 C"zprobe.in" H10 F2500 T6000         ; HOTEND PROBEset Z probe type to switch and the dive height + speeds
;M558 P5 I1 A4 R0.4 C"!^zprobe.in" H10 F250 T6000        ; FSR PROBE set Z probe type to switch and the dive height + speeds
G31 P500 X0 Y0 Z-0.4                                   ; set Z probe trigger value, offset and trigger height
M557 R130 S30                                           ; define mesh grid

; Bed Heater
M308 S0 P"bedtemp" Y"thermistor" T100000 B4725 C7.06e-8 ; configure sensor 0 as thermistor on pin bed temp
M950 H0 C"bedheat" T0                                   ; create bed heater output on bed heat and map it to sensor 0
M307 H0 R0.245 C774.3 D25.92 S1.00 V12.9                ; Bed Heater Process Parameters
M140 H0                                                 ; map heated bed to heater 0
M143 H0 S120                                            ; set temperature limit for heater 0 to 120C

; Hotend Heater
M308 S1 P"e0temp" Y"thermistor" T100000 B4725 C7.06e-8  ; configure sensor 1 as thermistor on pin e0temp
M950 H1 C"e0heat" T1                                    ; create nozzle heater output on e0heat and map it to sensor 1
M307 H1 R3.300 C110.0115.0 D7.00 S1.00                  ; Hotend Heater Process Parameters
M143 H1 S280                                            ; Hotend Max Temp

; Fans
M950 F0 C"fan0" Q500                                    ; create fan 0 on pin fan0 and set its frequency
M106 P0 S0 H-1                                          ; set fan 0 value. Thermostatic control is turned off
M950 F1 C"fan2" Q500                                    ; create fan 1 on pin fan1 and set its frequency
M106 P1 S0.80 H1 T45                                    ; set fan 1 value. Thermostatic control is turned on

; Tool Extruder 1
M563 P0 D0 H1 F0
G10 P0 X0 Y0 Z0
G10 P0 S0 R0

; Tool Extruder 2
M563 P1 D1 H1 F0
G10 P1 X0 Y0 Z0
G10 P1 S0 R0

;Filament Runout Sensor
M950 J0 C"!^e0Stop"                                     ; create switch pin
M950 J1 C"!^e1stop"                                     ; create switch pin
M581 P0:1 T2 S1 R1                                      ; run trigger2.g to pause if filament has run out during SD card printing

; Miscellaneous
M575 P1 S1 B57600                                       ; enable support for PanelDue
M501                                                    ; load saved parameters from non-volatile memory
T0                                                      ; select Tool 0
M911 S10.5 R11.2 P"M913 X0 Y0 G91 M83 G1 Z3 E-5 F1000"  ; set voltage thresholds and actions to run on power loss
M579 X1.0090 Y1.0090 Z1.0090                            ; scale axis all three must be equal
