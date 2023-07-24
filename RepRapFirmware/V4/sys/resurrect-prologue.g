M116           ; wait for temperatures
G28            ; home
M83            ; relative extrusion
G1 E4 F5000    ; undo the retraction that was done in the M911 power fail script