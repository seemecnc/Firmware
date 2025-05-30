;Prime Arc
G21 ; set units to millimeters
G90 ; use absolute coordinates
M82 ; use absolute distances for extrusion

G92 E0     ; zero extruder
;G1 E5 F300 ; prime 5mm prior to arc
;G92 E0     ; zero extruder

var radius = 143
var purge_se_xy = sqrt((var.radius*var.radius)/2)
var wipe_x = (var.radius/2)
var wipe_y = (var.wipe_x*sqrt(3))

; Do a 90deg purge arc with -15deg wipe/retract at R144.5
G1 X{-var.purge_se_xy} Y{-var.purge_se_xy} Z0.300 F18000           ; fast move to start point
G1 E5.00000 F1200.00000                                            ; extrude 5mm
G3 R{var.radius} X{var.purge_se_xy} Y{-var.purge_se_xy} E32 F1200  ; ccw arc 90 deg
G1 E31 F3000                                                       ; retract 1mm
G2 R{var.radius} X{var.wipe_x} Y{-var.wipe_y} E32 F9000            ; cw arc 15 deg - fast wipe/retract
G1 E30 F3000                                                       ; final retract
G1 Z2 F12000                                                       ; z-lift off purge line

G92 E0    ; zero extruder                             
M83; Relative Extruder mode
