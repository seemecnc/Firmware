; bed.g for SeeMeCNC 3D Printers - 300mm Bed
; Updated May 2026 for RRF v3.6.2+1
; Radius adjusted inward by 3mm (New Radius: 129mm)

M290 R0 S0                                  ; Reset baby steps
M561                                        ; Clear bed transform
M666 R0 X0 Y0 Z0                            ; Reset endstop offsets
M665 R{global.deltaRadius} L{global.armLength} B{global.bedRadius} H{global.homedHeight}   ; Set delta geometry

G28                                         ; Home
M117 Heating Bed Please Wait                
M190 S70                                    ; Bed preheat
T0                                          ; Select Tool 0
M104 S0                                     ; Hotend off
M140 S0                                     ; Bed off
G4 S1                                       ; Pause a second
G30 P0 X0.00 Y129.00 Z-99999 H0             ; Near Z tower
G30 P1 X-111.72 Y-64.50 Z-99999 H0          ; Near X tower
G30 P2 X111.72 Y-64.50 Z-99999 H0           ; Near Y tower
G30 P3 X111.72 Y64.50 Z-99999 H0            ; Mid Y-Z
G30 P4 X-111.72 Y64.50 Z-99999 H0           ; Mid X-Z
G30 P5 X0 Y-129.00 Z-99999 H0               ; Mid X-Y
G30 P6 X0 Y0 Z-99999 S6                     ; Center, 6-factor calibration
M500                                        ; Save settings
G28                                         ; Home