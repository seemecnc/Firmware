; bed.g for 300mm bed size SeeMeCNC 3D Printers
; updated March 16 2025
; called by G32 for 6-factor auto calibration with global variables
; 
; *** X Y Z AXIS MUST BE CALIBRATED FOR PROPER STEPS/MM OF MOVEMENT ***

M290 R0 S0                                  ; Reset baby steps
M561                                        ; Clear bed transform
M666 R0 X0 Y0 Z0                            ; Reset endstop offsets
M665 R{global.deltaRadius} L{global.armLength} B{global.bedRadius} H{global.homedHeight}   ; Set delta geometry from config.g global variables 
G28                                         ; Home
M117 Heating Bed Please Wait                ; Display message
M190 S70                                    ; Bed preheat
T0                                          ; Select Tool 0
M104 S0                                     ; Hotend off
M140 S0                                     ; Bed off
G4 S1                                       ; pause a second
G30 P0 X0.00 Y135.00 Z-99999 H0             ; Near Z tower
G30 P1 X-116.91 Y-67.5 Z-99999 H0           ; Near X tower
G30 P2 X116.91 Y-67.5 Z-99999 H0            ; Near Y tower
G30 P3 X116.91 Y67.5 Z-99999 H0             ; Mid Y-Z
G30 P4 X-116.91 Y67.5 Z-99999 H0            ; Mid X-Z
G30 P5 X0 Y-135 Z-99999 H0                  ; Mid X-Y
G30 P6 X0 Y0 Z-99999 S6                     ; Center, 6-factor calibration
M500                                        ; Save settings
G28                                         ; Home