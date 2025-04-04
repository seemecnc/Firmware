; bed.g for 300mm bed size SeeMeCNC 3D Printers
; updated March 16 2025
; called by G32 for 6-factor auto calibration with global variables
; 
; *** X Y Z AXIS MUST BE CALIBRATED FOR PROPER LENGTH OF MOVEMENT ***

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
G30 P0 X0.00 Y135.00 Z-99999 H0             ; Near Z tower
G30 P1 X132.95 Y23.44 Z-99999 H0            ; Near X tower
G30 P2 X-132.95 Y23.44 Z-99999 H0           ; Near Y tower
G30 P3 X46.17 Y-126.86 Z-99999 H0           ; Bottom-right
G30 P4 X-46.17 Y-126.86 Z-99999 H0          ; Bottom-left
G30 P5 X0.00 Y67.50 Z-99999 H0              ; Mid-top
G30 P6 X0 Y0 Z-99999 S6                     ; Center, 6-factor calibration
M500                                        ; Save settings
G28                                         ; Home