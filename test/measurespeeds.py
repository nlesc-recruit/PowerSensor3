import re
#import matplotlib.pyplot as plt
import numpy as np

imported_data = marker_found = None
marker_found = True
sensordata = [[],[],[],[],[],[],[]]

with open('output.txt', 'r') as reader:
  imported_data = reader.readlines()

markers = []
count = 0

for datum in imported_data:
    count += 1
    if marker_found is True:
        if re.search("S",datum):
            obj = re.search(" ",datum)
            rest = datum[obj.span()[1]:len(datum)]
            for i in range(6):
                obj = re.search(" ",rest)
                sensordata[i].append(float(rest[0:obj.span()[0]]))
                rest = rest[obj.span()[1]:len(rest)]
            sensordata[6].append(float(rest))
    if re.search("M",datum):
        markers.append(count)

print("processed imported data\n")
size = len(sensordata[0])
print(size)

highest_values = []

# find highest value
time = sensordata[0][10]
count = 0
stime = time + 1.0

samples = []

for datum in sensordata[0]:
    if datum > stime:
        samples.append(count)
        count = 0
        stime = datum + 1.0
    count += 1

#for i in range(7):
 #   print(sensordata[i][10000])

print(samples)

with open('input.txt', 'w+') as writer:
        for x in samples:
            writer.write(str(x))
            writer.write("\n")

exit()


print(highest_values)
size = len(sensordata[0]) 
print(size)

x_axis = np.arange(size)

start = 10
end = size

#y_axis = sensordata[6]

fig, ax = plt.subplots()
ax.set_ylim([0,30])

legendthings = ['','','12 V PSU', '12 V PSU', '12 V riser cable', '3.3 V riser cable', 'Sum']

#for i in range(2,7):
    #ax.plot(x_axis, y_axis, label=legendthings[i])

x_axis = x_axis[start:end]
#y_axis = y_axis[start:end]

#for i in range (2,7):
y_axis = sensordata[1]
y_axis = y_axis[start:end]
#ax.plot(x_axis, y_axis, label=legendthings[i])


#for value in highest_values:
 #  plt.axhline(value, c='r',linestyle='--')
 #   plt.text(end+130,value+2,str(int(value)))
    #for marker in markers:
#    plt.axvline(marker,0,250,c='grey',linestyle='--')

#plt.title('Power draw on individual sensors')
#plt.xlabel('samples', fontsize=12)
#plt.ylabel('power (w)', fontsize=12)

#plt.legend()
#plt.show()

#plt.savefig('variables.png')

