import matplotlib.pyplot as plt
txt_address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/BallVel4.txt"
vel = []
with open(txt_address, "r") as f:
    raw_data = f.readlines()
    for line in raw_data:
        line_data = line.split()
        if line_data[1] != '\n' and line_data[1] != '0':
            vel.append(line_data[1].strip())
vel = list(map(float, vel))
plt.plot(vel)
plt.show()