; bed.g
; called by G32 to perform auto calibration
;
M665 R142.5 L291.5 B145 H400                ;  Standard Injection Molded Arms 
G28                                         ; home
M117 Heating Bed Please Wait                ; display message
M190 S70                                    ; bed preheat temperature
M140 S0                                     ; turn off bed
M104 S0                                     ; hotend off
M30 "/sys/config-override.g"                ; delete
M561                                        ; clear any bed transform
G30 P0 X0 Y124.42 H0 Z-99999
G30 P1 X100.43 Y57.98 H0 Z-99999
G30 P2 X100.43 Y-57.98 H0 Z-99999
G30 P3 X0 Y-124.42 H0 Z-99999
G30 P4 X-108.25 Y-62.5 H0 Z-99999
G30 P5 X-108.25 Y62.5 H0 Z-99999
G30 P6 X0 Y61.46 H0 Z-99999
G30 P7 X46.77 Y-27 H0 Z-99999
G30 P8 X-54.13 Y-31.25 H0 Z-99999
G30 P9 X0 Y0 H0 Z-99999 S6
M500                                        ; save
G28
