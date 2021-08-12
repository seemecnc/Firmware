; bed.g
; called by G32 to perform auto calibration
;
M665 R150 L340.5 B150 H540                  ; Carbon Fiber ARMS - copy values from config.g !!
G28                                         ; home
M117 Heating Bed Please Wait                ; display message
M190 S70                                    ; bed preheat temperature
M140 S0                                     ; turn off bed
M104 S0                                     ; hotend off
M30 "/sys/config-override.g"                ; delete
M561                                        ; clear any bed transform
; Probe the bed at 3 peripheral and 3 halfway points, and perform 6-factor auto compensation
; Before running this, you should have set up your Z-probe trigger height to suit your build, in the G31 command in config.g.
G30 P0 X0 Y129.44 H0 Z-99999
G30 P1 X104.74 Y60.47 H0 Z-99999
G30 P2 X104.74 Y-60.47 H0 Z-99999
G30 P3 X0 Y-129.44 H0 Z-99999
G30 P4 X-112.58 Y-65 H0 Z-99999
G30 P5 X-112.58 Y65 H0 Z-99999
G30 P6 X0 Y63.99 H0 Z-99999
G30 P7 X48.9 Y-28.23 H0 Z-99999
G30 P8 X-56.29 Y-32.5 H0 Z-99999
G30 P9 X0 Y0 H0 Z-99999 S9
;M500                                        ; save
G28
; Use S-1 for measurements only, without calculations. Use S4 for endstop heights and Z-height only. Use S6 for full 6 factors
; If your Z probe has significantly different trigger heights depending on XY position, adjust the H parameters in the G30 commands accordingly. The value of each H parameter should be (trigger height at that XY position) - (trigger height at centre of bed)
