import matplotlib.pyplot as plt
txt_address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/BallVel1.txt"
raw_id = []
raw_vel = []
raw_dist = []
avg_vel = []
with open(txt_address, "r") as f: # 读取数据
    raw_data = f.readlines()
    for line in raw_data:
        line_data = line.split()
        if line_data[1] != '\n' and line_data[1] != '0':
            raw_vel.append(line_data[1].strip())
raw_vel = list(map(float, raw_vel))
per = 5 # 均值滤波
avg_len = len(raw_vel) / per
for i in range(0, len(raw_vel)-per, per):
    sum = 0
    for j in range(per):
        sum += raw_vel[i+j]
    for j in range(per):
        avg_vel.append(sum/per)
max_vel = max(avg_vel) # 求最值
max_index = avg_vel.index(max_vel)
chip_point = -1 #　找落地点
for i in range(max_index, len(avg_vel)-1):
    if avg_vel[i] < avg_vel[i+1]:
        chip_point = i
        break
plt.plot(raw_vel, color='green')
plt.plot(chip_point, raw_vel[chip_point], 'o', color='black')
plt.plot(avg_vel, color='red')
plt.show()