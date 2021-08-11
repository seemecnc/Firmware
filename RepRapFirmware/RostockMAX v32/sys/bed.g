; bed.g
; called by G32 to perform auto calibration
;
M665 R150 L340.5 B150 H400                  ; Carbon Fiber ARMS - copy values from config.g !!
G28                                         ; home
M117 Heating Bed Please Wait                ; display message
M190 S70                                    ; bed preheat temperature
M140 S0                                     ; turn off bed
M104 S0                                     ; hotend off
M30 "/sys/config-override.g"                ; delete
M561                                        ; clear any bed transform
G30 P0 X0.00 Y130.00 Z-99999 H0             ; Probe points
G30 P1 X83.56 Y99.59 Z-99999 H0
G30 P2 X128.03 Y22.57 Z-99999 H0
G30 P3 X112.58 Y-65.00 Z-99999 H0
G30 P4 X44.46 Y-122.16 Z-99999 H0
G30 P5 X-44.46 Y-122.16 Z-99999 H0
G30 P6 X-112.58 Y-65.00 Z-99999 H0
G30 P7 X-128.03 Y22.57 Z-99999 H0
G30 P8 X-83.56 Y99.59 Z-99999 H0
G30 P9 X0.00 Y65.00 Z-99999 H0
G30 P10 X56.29 Y-32.50 Z-99999 H0
G30 P11 X-56.29 Y-32.50 Z-99999 H0
G30 P12 X0 Y0 Z-99999 S9                    ; 9 factor calibration
M500
G28                                         ; home
G30 P0 X0.00 Y130.00 Z-99999 H0             ; Probe points
G30 P1 X83.56 Y99.59 Z-99999 H0
G30 P2 X128.03 Y22.57 Z-99999 H0
G30 P3 X112.58 Y-65.00 Z-99999 H0
G30 P4 X44.46 Y-122.16 Z-99999 H0
G30 P5 X-44.46 Y-122.16 Z-99999 H0
G30 P6 X-112.58 Y-65.00 Z-99999 H0
G30 P7 X-128.03 Y22.57 Z-99999 H0
G30 P8 X-83.56 Y99.59 Z-99999 H0
G30 P9 X0.00 Y65.00 Z-99999 H0
G30 P10 X56.29 Y-32.50 Z-99999 H0
G30 P11 X-56.29 Y-32.50 Z-99999 H0
G30 P12 X0 Y0 Z-99999 S9                    ; 9 factor calibration
M500
M140 S0                                     ; bed off
G28                                         ; home


