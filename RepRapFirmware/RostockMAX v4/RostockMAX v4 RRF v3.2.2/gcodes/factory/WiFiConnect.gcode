M552 S0                                    ; wifi off
G4 P1000                                   ; delay
M587 S"networkSSID" P"password"            ; replace with your WiFi information
M552 S1                                    ; turn on wifi
