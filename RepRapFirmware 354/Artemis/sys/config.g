; SeeMeCNC 3D Printers Config 
; JULY 2025
; This release is tailored for Orca Slicer Software, but will also work with Cura 4.6.2
; ***important*** Line M92 Steps/mm on all axis must be calibrated manually.
;
; General Settings
G90                                 ; absolute coordinates
M83                                 ; relative extruder moves
M550 P"ARTEMIS"                     ; set printer name (ARTEMIS, RostockMAX, BOSSdelta, SeeMeCNC, BestFriend, etc.)

; Set Global Variables
global T_bed_0    = 0               ; {material_bed_temperature_layer_0}
global T_tool_0   = 0               ; {material_print_temperature_layer_0}
global T_bed      = 0               ; {material_bed_temperature}
global T_tool     = 0               ; {material_print_temperature}
global bed_type = "Unknown"			; build plate type from Orca for mesh/offset saving
global filament_type = "Unknown"    ; filament type for offset saving

; Global M665 Variables
global deltaRadius = 160            ; Horizontal radius (R) 160=CF arms and SMC carriages;  155=CF Arms and Linear Rails;  150=CF Arms and Linear Rails
global armLength = 340.5            ; Diagonal rod length (L) 340.5=CF Arms; 351.1=Injection Molded Arms
global bedRadius = 155              ; Printable bed radius (B) 155=CF Arms; 145=Injection Molded Arms
global homedHeight = 540            ; Homed height (H) 540=CF Arms; 530=Injection Molded Arms
M665 R{global.deltaRadius} L{global.armLength} B{global.bedRadius} H{global.homedHeight}  ; Delta Parameters
M666 X0 Y0 Z0 A0 B0                 ; endstop adjustment (this is set by autocalibration leveling)
M208 Z0 S1                          ; set Z axis minimum limit
M208 Z{global.homedHeight} S1                           ; set maximums

; Network & Comms
M540 PF0:E1:D2:C3:11:11	                                ; Set the printers MAC Address (make different for each SeeMeCNC printer you own)
M586 P0 S1                                              ; enable HTTP
G4 P500                                                 ; pause
M552 S1                                                 ; enable network and acquire dynamic address via DHCP
G4 P500                                                 ; pause
M586 P1 S0                                              ; disable FTP
M586 P2 S0                                              ; disable Telnet
M575 P1 S1 B57600                                       ; PanelDue Comm Setup

; Drive Parameters
M569 P0 S0                                              ; physical drive 0
M569 P1 S0                                              ; physical drive 1
M569 P2 S0                                              ; physical drive 2
M569 P3 S1                                              ; physical drive 3
M569 P4 S1                                              ; physical drive 4
M584 X0 Y1 Z2 E3:4                                      ; set drive mapping
M350 X16 Y16 Z16 E16:16 I1                              ; configure micro stepping with interpolation
M92 X200.00 Y200.00 Z200.00 E182.00:182.00              ; set steps per mm - ALL MUST BE CALIBRATED
M566 X1500.00 Y1500.00 Z1500.00 E1500.00:1500.00        ; set maximum instantaneous speed changes (mm/min) aka Jerk
M203 X12000.00 Y12000.00 Z12000.00 E9000.00:9000.00     ; set maximum speeds (mm/min)
M201 X2000.00 Y2000.00 Z2000.00 E3000.00:3000.00        ; set accelerations (mm/s^2)
M906 X1600 Y1600 Z1600 E1600:1600 I30                   ; set motor currents (mA) and motor idle factor in per cent
M84 S30                                                 ; Set motor idle timeout (seconds)

; End-stops
M574 X2 S1 P"xstop"                                     ; configure active-high endstop for high end on X via pin xstop
M574 Y2 S1 P"ystop"                                     ; configure active-high endstop for high end on Y via pin ystop
M574 Z2 S1 P"zstop"                                     ; configure active-high endstop for high end on Z via pin zstop

; Z-Probe
M558 P5 I0 A4 R0.4 C"zprobe.in" H20 F2500 T6000         ; HOTEND PROBEset Z probe type to switch and the dive height + speeds
G31 P500 X0 Y0 Z-0.4                                    ; set Z probe trigger value, offset and trigger height
M557 R130 S30                                           ; define mesh grid

; Bed Heater
M308 S0 P"bedtemp" Y"thermistor" A"Heated Bed" T100000 B4725 C7.06e-8 ; configure sensor 0 as thermistor on pin bed temp
M950 H0 C"bedheat" T0                                   ; create bed heater output on bed heat and map it to sensor 0
M307 H0 R0.125 K0.170:0.000 D35.00 E1.35 S1.00 B0       ; Bed Heater Process Parameters Tuning M303 H0 P1 S80
M140 H0                                                 ; map heated bed to heater 0
M143 H0 P0 T0 C0 S120 A0                                ; set temperature limit for heater 0 to 120C

; Hotend Heater
M308 S1 P"e0temp" Y"thermistor" A"Hotend" T100000 B4725 C7.06e-8  ; configure sensor 1 as thermistor on pin e0temp
M950 H1 C"e0heat" T1                                    ; create nozzle heater output on e0heat and map it to sensor 1
M307 H1 R3.500 K0.650:0.300 D8.00 E1.35 S1.00 B0 V13.0  ; Example Heater Tuning Command  M303 H1 P1 S250
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

; pressure advance is now inside of Orca Slicer FILAMENT Settings 
;M572 D0:1 S0.12:0.12  DO NOT ENABLE - Kept for reference only

; Input Shaping
;M593 P"ei3" F28 S0.1                                   ; not tuned but kept for future reference
M593 P"mzv" F25 S0                                      ; not precisely tuned

;Filament Runout Sensor
M950 J0 C"!^e0Stop"                                     ; create switch pin
M950 J1 C"!^e1stop"                                     ; create switch pin
M581 P0:1 T2 S1 R1                                      ; run trigger2.g to pause if filament has run out during SD card printing

; Miscellaneous
M501                                                    ; load saved parameters from non-volatile memory
T0                                                      ; select Tool 0
M911 S10.9 R11.2 P"M913 X0 Y0 G91 M83 G1 Z3 E-5 F1000"  ; set voltage thresholds and actions to run on power loss
;M579 X1.0000 Y1.0000 Z1.0000   ; Scale is now filament Shrink in Orca Slicer Filament Settings
