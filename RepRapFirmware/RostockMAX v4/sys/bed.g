; bed.g
; called by G32 to perform auto calibration
;
M665 R150 L340.5 B150 H400                  ; Carbon Fiber ARMS - copy values from config.g !!
G28                                         ; home
M117 Heating Bed Please Wait                ; display message
M190 S70                                    ; bed preheat temperature
M104 S0                                     ; hotend off
M30 "/sys/config-override.g"                ; delete
M561                                        ; clear any bed transform
G30 P0 X0.00 Y125.00 Z-99999 H0             ; Probe points
G30 P1 X80.35 Y95.76 Z-99999 H0
G30 P2 X123.10 Y21.71 Z-99999 H0
G30 P3 X108.25 Y-62.50 Z-99999 H0
G30 P4 X42.75 Y-117.46 Z-99999 H0
G30 P5 X-42.75 Y-117.46 Z-99999 H0
G30 P6 X-108.25 Y-62.50 Z-99999 H0
G30 P7 X-123.10 Y21.71 Z-99999 H0
G30 P8 X-80.35 Y95.76 Z-99999 H0
G30 P9 X0.00 Y62.50 Z-99999 H0
G30 P10 X54.13 Y-31.25 Z-99999 H0
G30 P11 X-54.13 Y-31.25 Z-99999 H0
G30 P12 X0 Y0 Z-99999 S9                    ; 9 factor calibration
M500
G28                                         ; home
G30 P0 X0.00 Y125.00 Z-99999 H0             ; Probe points
G30 P1 X80.35 Y95.76 Z-99999 H0
G30 P2 X123.10 Y21.71 Z-99999 H0
G30 P3 X108.25 Y-62.50 Z-99999 H0
G30 P4 X42.75 Y-117.46 Z-99999 H0
G30 P5 X-42.75 Y-117.46 Z-99999 H0
G30 P6 X-108.25 Y-62.50 Z-99999 H0
G30 P7 X-123.10 Y21.71 Z-99999 H0
G30 P8 X-80.35 Y95.76 Z-99999 H0
G30 P9 X0.00 Y62.50 Z-99999 H0
G30 P10 X54.13 Y-31.25 Z-99999 H0
G30 P11 X-54.13 Y-31.25 Z-99999 H0
G30 P12 X0 Y0 Z-99999 S9                    ; 9 factor calibration
M500
M140 S0                                     ; bed off
G28                                         ; home


