; Basic Pickup script
; 11/03/2023

; Setup
G28; Measure
G100 A0 B1.57 C-1.57 D0

T1; Select Axisbearing tool
M5; Make sure that the tong is deactivated

; Programm
G100 A0 B1.3 C-1.57 D-1.57; Drive to lower position

G4 X0.5
M3; Close tongs
G4 X0.5

G100 A0 B1.57 C-1.57 D-1.57; Lift
M119 A0.787
G100 A0 B1.3 C-1.57 D-1.57; Put down

M5; Open Tongs

G28; Start Position