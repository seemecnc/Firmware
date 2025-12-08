; cancel g-code
; SeeMeCNC July 2023 update cancel 
;
G91                          ; incremental
G1 Z2 F8000                  ; raise Z
M104 S0 T1                   ; hotend off
M104 S0 T0                   ; hotend off
M140 S0                      ; bed off
M106 S0                      ; fan off
M203 Z10000                  ; set speed (this compensated for old control problem)
G92 E0                       ; zero extruder
G1 E-10 F1000                ; retract filament to starting position
G92 E0                       ; zero extruder
G1 E-150 F5000               ; retract filament to starting position
G92 E0                       ; zero extruder
M84 E0:1                     ; extruder motors off
G28                          ; home
