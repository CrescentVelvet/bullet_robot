import math

# with open("/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/ReguDataRotate_all.txt", "r") as f:
#     raw_id = []
#     raw_div = []
#     raw_minus = []
#     raw_rotnow = []
#     raw_rotlim = []
#     raw_dircar = []
#     raw_data = f.readlines()
#     for line in raw_data:
#         line_data = line.split()
#         if len(line_data) != 6:
#             continue
#         if line_data[0] != '\n':
#             raw_id.append(float(line_data[0].strip()))
#         if line_data[1] != '\n':
#             raw_div.append(float(line_data[1].strip()))
#         if line_data[2] != '\n':
#             raw_minus.append(float(line_data[2].strip()))
#         if line_data[3] != '\n':
#             raw_rotnow.append(float(line_data[3].strip()))
#         if line_data[4] != '\n':
#             raw_rotlim.append(float(line_data[4].strip()))
#         if line_data[5] != '\n':
#             raw_dircar.append(float(line_data[5].strip()))
# with open("/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/newReguDataRotate.txt", "w") as f:
#     for i in range(len(raw_id)):
#         f.write(' ')
#         f.write(str(raw_id[i]))
#         f.write(' ')
#         f.write(str(math.tan(raw_div[i])))
#         f.write(' ')
#         f.write(str(math.tan(math.atan(raw_minus[i]+raw_dircar[i])-raw_dircar[i])))
#         f.write(' ')
#         f.write(str(raw_rotnow[i]))
#         f.write(' ')
#         f.write(str(raw_rotlim[i]))
#         f.write(' ')
#         f.write(str(raw_dircar[i]))
#         f.write('\n')
with open("/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/ReguDataRotate_all.txt", "r") as f:
    raw_0 = []
    raw_1 = []
    raw_2 = []
    raw_3 = []
    raw_4 = []
    raw_5 = []
    raw_data = f.readlines()
    for line in raw_data:
        line_data = line.split()
        if len(line_data) != 6:
            continue
        if line_data[0] != '\n':
            raw_0.append(float(line_data[0].strip()))
        if line_data[1] != '\n':
            raw_1.append(float(line_data[1].strip()))
        if line_data[2] != '\n':
            raw_2.append(float(line_data[2].strip()))
        if line_data[3] != '\n':
            raw_3.append(float(line_data[3].strip()))
        if line_data[4] != '\n':
            raw_4.append(float(line_data[4].strip()))
        if line_data[5] != '\n':
            raw_5.append(float(line_data[5].strip()))
with open("/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/newReguDataRotate.txt", "w") as f:
    for i in range(len(raw_0)):
        f.write(' ')
        f.write(str(raw_0[i]))
        f.write(' ')
        f.write(str(math.atan(math.atan(raw_1[i]))))
        f.write(' ')
        f.write(str(math.atan(raw_2[i])))
        f.write(' ')
        f.write(str(raw_3[i]))
        f.write(' ')
        f.write(str(raw_4[i]))
        f.write(' ')
        f.write(str(raw_5[i]))
        f.write('\n')