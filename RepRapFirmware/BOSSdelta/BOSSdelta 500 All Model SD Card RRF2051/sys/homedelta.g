; homedelta.g
G91                             ; relative positioning
G1 S1 X1250 Y1250 Z1250 F5000   ; move all carriages up stopping at the endstops
G1 S2 X-5 Y-5 Z-5 F500          ; move all towers down
G1 S1 X8 Y8 Z8 F500             ; move towers up slowly touching endstops
G1 S2 X-5 Y-5 Z-5 F500          ; move carriages down
G90                             ; absolute positioning
