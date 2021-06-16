; homedelta.g
G91                             ; relative positioning
G1 H1 X1250 Y1250 Z1250 F5000   ; move all towers to the high end stopping at the end-stops (first pass)
G1 H2 X-5 Y-5 Z-5 F5000         ; go down a few mm
G1 H1 X10 Y10 Z10 F500          ; move all towers up once more (second pass)
G1 Z-5 F1000                    ; move down a few mm so that the nozzle can be centered
G90                             ; absolute positioning
