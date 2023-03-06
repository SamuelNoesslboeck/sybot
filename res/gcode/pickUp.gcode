; Basic Pickup script
; 06/03/2023

; Setup
G28; Measure
T1; Select Axisbearing tool
M5; Make sure that the spindle is deactivated

; Program
G0 X-50 Y400 Z50; Drive to low position
M3; Grab

G0 X-50 Y400 Z50; Lift object
G0 X50 Y400 Z50; Move object to the right

G0 X50 Y400 Z0; Put object down
M5; Release

M30; Exit program