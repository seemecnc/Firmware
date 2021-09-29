; resume.g
; called before a print from SD card is resumed
G1 R1 X0 Y0 Z20 F3000   ; move above last print move position
M83                     ; relative extruder moves
G1 E10 F3600            ; extrude 10mm of filament