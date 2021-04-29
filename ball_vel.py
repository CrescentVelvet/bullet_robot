import matplotlib.pyplot as plt
txt_address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/BallVel3.txt"
raw_vel = []
diff_vel = []
avg_vel = []
with open(txt_address, "r") as f:
    raw_data = f.readlines()
    for line in raw_data:
        line_data = line.split()
        if line_data[1] != '\n' and line_data[1] != '0':
            raw_vel.append(line_data[1].strip())
raw_vel = list(map(float, raw_vel))
# real_vel = [i for i in raw_vel if i > 1500]
max_vel = max(raw_vel) # 求最值
max_index = raw_vel.index(max_vel)
chip_point = -1
for i in range(len(raw_vel)): #　找落地点
    if i < max_index:
        continue
    if raw_vel[max_index] - raw_vel[i] > 5000:
        chip_point = i
        break
for i in range(len(raw_vel)-1): # 求差分
    diff_vel.append(raw_vel[i+1] - raw_vel[i])
per = 5 # 均值滤波
avg_len = len(raw_vel) / per
for i in range(0, len(raw_vel)-per, per):
    sum = 0
    for j in range(per):
        sum += raw_vel[i+j]
    for j in range(per):
        avg_vel.append(sum/per)
    # avg_vel.append(raw_vel, raw_vel[len(raw_vel)-1])
# print(diff_vel)
plt.plot(raw_vel, color='green')
plt.plot(chip_point, raw_vel[chip_point], 'o', color='red')
plt.plot(avg_vel, color='red')
plt.show()