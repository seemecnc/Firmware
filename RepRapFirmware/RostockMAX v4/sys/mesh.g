; mesh.g
M117 Bed Map - Heating Bed                              ; display message
G28                                                     ; home
M140 S70                                                ; set bed temp 
M190 S70                                                ; wait for bed temp
M140 S0                                                 ; bed heat off
M558 P5 I0 A5 R0.4 C"zprobe.in" H10 F1700 T6000 S0.06   ; HOTEND PROBE set Z probe type to switch and the dive height + speeds
M557 R125 S30                                           ; HOTEND PROBE define mesh grid
G29 S2                                                  ; clear height map
G29 S0                                                  ; run mesh calibration
M500                                                    ; save results to EEPROM
G28                                                     ; home
;M375                                                    ; load heightmap
M84                                                     ; motors off
M117 End Mesh Probe Calibration                         ; display message