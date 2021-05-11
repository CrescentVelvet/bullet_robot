
with open("/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/FlatData.txt", "r") as f:
    raw_id = []
    raw_vel = []
    raw_maxvel = []
    raw_power = []
    raw_data = f.readlines()
    for line in raw_data:
        line_data = line.split()
        if line_data[1] == '0' or line_data[2] == '-1' or line_data[2] == '-1000': # 排除初始化数据
            continue
        if line_data[0] != '\n':
            raw_id.append(line_data[0].strip())
        if line_data[1] != '\n':
            raw_vel.append(line_data[1].strip())
        if line_data[2] != '\n':
            raw_maxvel.append(line_data[2].strip())
        if line_data[4] != '\n':
            raw_power.append(line_data[4].strip())
with open("/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/newFlatData.txt", "w") as f:
    for i in range(len(raw_id)):
        f.write(' ')
        f.write(raw_id[i])
        f.write(' ')
        f.write(raw_vel[i])
        f.write(' ')
        f.write(raw_maxvel[i])
        f.write(' ')
        f.write(raw_power[i])
        f.write('\n')