G91                             ; use relative positioning
G1 H1 X1250 Y1250 Z1250 F5000   ; move all carriages up stop at the endstops
G1 H2 X-5 Y-5 Z-5 F500          ; move all towers down some
G1 H1 X8 Y8 Z8 F500             ; move towers slowly up stopping at the endstops
G1 Z-5 F3000
G90                             ; back to absolute positioning
G1 X0 Y0 F6000                ; move X+Y to the centre
