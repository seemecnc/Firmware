; resume.g
; updated July 2023
; called before a print from SD card is resumed
; updated July 2023
G1 R1 X0 Y0 Z30 F8000 ; move above last print move position
M83                   ; relative extruder moves
G1 E10 F5000          ; extrude 10mm of filament
