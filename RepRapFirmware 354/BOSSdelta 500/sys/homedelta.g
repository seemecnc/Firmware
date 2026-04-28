; SeeMeCNC July 2025
; homed height variable is set in config.g 
; change error in line above G90 - April 2026
G91                                                                             ; relative positioning
G1 H1 X{global.homedHeight} Y{global.homedHeight} Z{global.homedHeight} F5000   ; initial home move to endstop quickly
G1 H1 X{global.homedHeight} Y{global.homedHeight} Z{global.homedHeight}         ; continue homing axis not yet touching
G1 H2 X-5 Y-5 Z-5 F5000                                                         ; go down a few mm
G1 H1 X{global.homedHeight} Y{global.homedHeight} Z{global.homedHeight} F500    ; slow home move for precision
G1 H2 X-5 Y-5 Z-5 F500                                                          ; move down off endstop
G90                                                                             ; absolute positioning