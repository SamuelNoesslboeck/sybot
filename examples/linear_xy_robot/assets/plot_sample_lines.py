import matplotlib.pyplot as plt
import json

lines_file = json.loads(open("sample_lines.json").read())
lines = lines_file["contour"]

print(f"Loaded {len(lines)} lines")

x1 = [ line["p1"][0] for line in lines ]
y1 = [ line["p1"][1] for line in lines ]
x2 = [ line["p2"][0] for line in lines ]
y2 = [ line["p2"][1] for line in lines ]

plt.plot(x1, y1, x2, y2)
plt.show() 