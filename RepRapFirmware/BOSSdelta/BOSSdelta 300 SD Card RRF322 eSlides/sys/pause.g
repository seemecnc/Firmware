; pause.g
; called when a print from SD card is paused
M83                ; relative extruder moves
G91                ; relative positioning
G1 Z20             ; lift Z by 5mm
G90                ; absolute positioning
M84 E0:1           ;turn off extruder motors
