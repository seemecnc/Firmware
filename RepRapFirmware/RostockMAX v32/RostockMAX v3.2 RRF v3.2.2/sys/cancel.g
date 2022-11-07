M104 S0                      ; hotend off
M140 S0                      ; bed off
M106 S0                      ; fan off
M203 Z10000                  ; set speed (this compensated for old control problem)
G92 E0                       ; zero extruder
G1 E-160 F5000               ; retract filament to starting position
G92 E0                       ; zero extruder
G28
;M84                         ; do not use motor off command
