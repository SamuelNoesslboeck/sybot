; A little fist bump, just to test a few things
; 18/12/2022
G28; Measure

G8 X-400 Y0 Z300; Position behind
G4 X1           ; Wait 1 sec
G8 X-520 Y0 Z300; Position front
G4 X1           ; Wait 1 sec
G0 X-400 Y0 Z300; Position behind

G8 X0 Y380 Z400 ; Correct home position

G28; Back to home