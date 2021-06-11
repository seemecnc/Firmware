M550 PDO_NOT_USE                           ; Printer name  ONLY USED FOR FIRMWARE UPGRADE
M555 P2                                    ; Gcode Output Type
M552 S0                                    ; DISable Wifi
M575 P1 B57600 S1                          ; PanelDue Comm Setup
G21                                        ; Work in millimetres
G90                                        ; Send absolute coordinates

M569 P0 S0                                 ; Drive 0 goes forwards (X)
M569 P1 S0                                 ; Drive 1 goes forwards (Y)
M569 P2 S0                                 ; Drive 2 goes forwards (Z)
M569 P3 S0                                 ; Drive 3 goes forwards (E0)
M569 P4 S0                                 ; Drive 4 goes forwards (E1)

M574 X2 Y2 Z2 S1                           ; set endstop configuration (all endstops at high end, active high)

M665 R144 L291.06 B135 H400 X0 Y0 Z0       ; delta radius, diagonal rod length, printable radius and homed height
;M665 R144 L337 B135 H350 X0 Y0 Z0         ; optional carbon fiber arms length setting
                                           ; Y X Z are tower angle offsets
M666 X0 Y0 Z0                              ; endstop offsets in mm

M350 X16 Y16 Z16 E16:16 I1                 ; Set 16x microstepping w/ Interpolation
M92 X200 Y200 Z200                         ; Set axis steps/mm
M92 E182.0:182.0                           ; Set extruder steps/mm

M906 X1200 Y1200 Z1200 E1200:1200 I50      ; Set motor currents (mA) and idle current %
M201 X4200 Y4200 Z4200 E5000               ; Accelerations (mm/s^2)
M203 X15000 Y15000 Z15000 E15000           ; Maximum speeds (mm/min)
M566 X2000 Y2000 Z2000 E2000               ; Maximum instant speed changes mm/minute

M106 P0 H-1                                ; Part Cooling Fan
M106 P1 S0.5 H-1                           ; Case fan
M106 P2 T50 S0.7 H1                        ; Heat sink fan

M307 H0 B0                                 ; Heated Bed (H2)
M305 P0 T100000 B4388 R4700 H30 L0         ; Bed thermistor

M305 P1 T100000 B4388 R4700 C7.06e-8 H30 L0      ; Hotend Thermistor
M563 P0 D0 H1                              ; Hot end (T0), drive (E0), heater (H1)
G10 P0 S0 R0                               ; Hot end operating and standby temperatures
