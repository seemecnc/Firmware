; pause.g
; called when a print is paused
; updated July 2023
M83                ; relative extruder moves
G91                ; relative positioning
G1 E-3 F5000       ; fast retract filament
G1 E-7 F5000       ; slow retract
G1 Z30 F8000       ; lift Z
G90                ; absolute positioning
M84 E0:1           ; turn off extruder motors
