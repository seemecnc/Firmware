; bed.g BOSS 500 series
; called by G32 to perform auto calibration or pressing wavy icon on touchscreen
;
M665 R260 L572.92 B260 H1130                ; delta radius, diagonal rod length, printable radius and homed height
G28                                         ; home
M117 Heating Bed Please Wait                ; display message
M190 S70                                    ; bed preheat temperature
M104 S0                                     ; hotend off
M140 S0                                     ; bed off
M30 "/sys/config-override.g"                ; delete
M30 "/sys/heightmap.csv"                    ; delete
M561                                        ; clear any bed transform
G30 P0 X0 Y244.9 H0 Z-99999
G30 P1 X212.09 Y122.45 H0 Z-99999
G30 P2 X212.09 Y-122.45 H0 Z-99999
G30 P3 X0 Y-244.9 H0 Z-99999
G30 P4 X-212.09 Y-122.45 H0 Z-99999
G30 P5 X-212.09 Y122.45 H0 Z-99999
G30 P6 X0 Y122.4 H0 Z-99999
G30 P7 X106 Y61.2 H0 Z-99999
G30 P8 X106 Y-61.2 H0 Z-99999
G30 P9 X0 Y-122.4 H0 Z-99999
G30 P10 X-106 Y-61.2 H0 Z-99999
G30 P11 X-106 Y61.2 H0 Z-99999
G30 P12 X0 Y0 H0 Z-99999 S9
M500
G28                                         ; home
