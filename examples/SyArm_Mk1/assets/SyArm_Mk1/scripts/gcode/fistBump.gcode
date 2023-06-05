; A little fist bump, just to test a few things
; 18/12/2022
G28; Measure

; Fistbump program
G8 X-400 Y0 Z300    ; Position behind
G4 P200             ; Wait 1 sec
G8 X-460 Y0 Z300    ; Position front
G8 X-400 Y0 Z300    ; Position behind

G8 X0 Y380 Z400     ; Correct home position

G29; Back to home