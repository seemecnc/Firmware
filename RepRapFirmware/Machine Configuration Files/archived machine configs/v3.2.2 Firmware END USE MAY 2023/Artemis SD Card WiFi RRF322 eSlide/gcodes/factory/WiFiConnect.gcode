M552 S0                                    ; wifi off
G4 P1000                                   ; delay
M587 S"seemecnc" P"seeme3dp"               ;replace with your WiFi information
;M587 S"networkSSID" P"password"           ;example
M552 S1                                    ; turn on wifi
