import re
import matplotlib.pyplot as plt
import numpy as np

imported_data = marker_found = None
marker_found = True
sensordata = [[],[],[],[],[],[],[]]

y_highest = []
temp = [0,0,0,0,0,0,0,0]

f = open('output3.txt', 'r')
for line in f:
    obj = line.split()
    print(obj)
    for i in range (3,8):
        if float(obj[i]) > temp[i]:
            temp[i] = float(obj[i])
            

print(temp)

exit()


imported_data = reader.readlines()

markers = []
count = 0


for datum in imported_data:
    if count == 1:
        count = 0
        if re.search("S",datum):
            obj = re.search(" ",datum)
            rest = datum[obj.span()[1]:len(datum)]
            for i in range(6):
                obj = re.search(" ",rest)
                sensordata[i].append(float(rest[0:obj.span()[0]]))
                rest = rest[obj.span()[1]:len(rest)]
            sensordata[6].append(float(rest))
    count += 1

print("processed imported data\n")
size = len(sensordata[0])
print(size)

y_highest = []
x_corr = []
temp = 0
tempx = 0

for j in range(size):
    if sensordata[6][j] > temp:
        temp = sensordata[6][j]
        tempx = j

y_highest.append(temp)
x_corr.append(tempx)
temp = 0

print(y_highest)
print(x_corr)

exit()

x_axis = np.arange(size)

start = 0
end = size

fig, ax = plt.subplots()
ax.set_ylim([0,250])

legendthings = ['','','12 V PSU', '12 V PSU', '12 V riser cable', '3.3 V riser cable', 'Sum']

for i in range(2,7):
    ax.plot(x_axis[start:end], sensordata[i][start:end], label=legendthings[i])

for i in range(len(y_highest)):
    ax.plot(x_corr[i], y_highest[i], 'r+')
    plt.text(x_corr[i], y_highest[i], str(int(y_highest[i])))

plt.title('Power draw on individual sensors')
plt.xlabel('samples', fontsize=12)
plt.ylabel('power (w)', fontsize=12)
plt.grid(True)

plt.legend()
#plt.show()

plt.savefig('kerneltunerresults.png')

