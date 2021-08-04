; bed.g
; called by G32 to perform auto calibration
;
M665 R150 L340.5 B150 H510                  ; Carbon Fiber ARMS - copy values from config.g !!
G28                                         ; home
M117 Heating Bed Please Wait                ; display message
M190 S70                                    ; bed preheat temperature
M104 S0                                     ; hotend off
M30 "/sys/config-override.g"                ; delete
M561                                        ; clear any bed transform
G30 P0 X0.00 Y135.00 Z-99999 H0             ; Probe points
G30 P1 X86.78 Y103.42 Z-99999 H0
G30 P2 X132.95 Y23.44 Z-99999 H0
G30 P3 X116.91 Y-67.50 Z-99999 H0
G30 P4 X46.17 Y-126.86 Z-99999 H0
G30 P5 X-46.17 Y-126.86 Z-99999 H0
G30 P6 X-116.91 Y-67.50 Z-99999 H0
G30 P7 X-132.95 Y23.44 Z-99999 H0
G30 P8 X-86.78 Y103.42 Z-99999 H0
G30 P9 X0.00 Y67.50 Z-99999 H0
G30 P10 X58.46 Y-33.75 Z-99999 H0
G30 P11 X-58.46 Y-33.75 Z-99999 H0
G30 P12 X0 Y0 Z-99999 S9                    ; 9 factor calibration
M500
G28                                         ; home
G30 P0 X0.00 Y135.00 Z-99999 H0             ; Probe points
G30 P1 X86.78 Y103.42 Z-99999 H0
G30 P2 X132.95 Y23.44 Z-99999 H0
G30 P3 X116.91 Y-67.50 Z-99999 H0
G30 P4 X46.17 Y-126.86 Z-99999 H0
G30 P5 X-46.17 Y-126.86 Z-99999 H0
G30 P6 X-116.91 Y-67.50 Z-99999 H0
G30 P7 X-132.95 Y23.44 Z-99999 H0
G30 P8 X-86.78 Y103.42 Z-99999 H0
G30 P9 X0.00 Y67.50 Z-99999 H0
G30 P10 X58.46 Y-33.75 Z-99999 H0
G30 P11 X-58.46 Y-33.75 Z-99999 H0
G30 P12 X0 Y0 Z-99999 S9                    ; 9 factor calibration
M500
M140 S0                                     ; bed off
G28                                         ; home

