import re
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.axes_grid.inset_locator import (inset_axes, InsetPosition, mark_inset)


imported_data = [[],[],[],[],[]]

with open('input1.txt', 'r') as reader:
  imported_data[0] = reader.readlines()

with open('input2.txt', 'r') as reader:
  imported_data[1] = reader.readlines()

with open('input3.txt', 'r') as reader:
  imported_data[2] = reader.readlines()

with open('input4.txt', 'r') as reader:
  imported_data[3] = reader.readlines()

with open('input5.txt', 'r') as reader:
  imported_data[4] = reader.readlines()

x_axis = np.arange(1,20)

sampledata = [[],[],[],[],[]]
average = []


for i in range(5):
    for j in range(0,19):
        sampledata[i].append(int(imported_data[i][j]))

value = 0

for i in range (0,19):
    for j in range (5):
        value += sampledata[j][i] 
    average.append(value/5)
    value = 0


print(sampledata)

fig, ax = plt.subplots()
ax.set_ylim([160000,200001])
plt.yticks(np.arange(160000,200001,step=10000))
ax.yaxis.set_minor_locator(plt.MultipleLocator(2500))


legendthings = ['Test #1', 'Test #2', 'Test #3', 'Test #4', 'Test #5']
colors = ['b','g','c','m','y']


#for i in range(2,7):
    #ax.plot(x_axis, y_axis, label=legendthings[i])

#y_axis = y_axis[start:end]

#for i in range (2,7):
#y_axis = sensordata[]
#y_axis = y_axis[start:end]
#ax.plot(x_axis, y_axis, label=legendthings[i])

begin = 0
end = 10

for i in range(5):
    ax.plot(x_axis[begin:end], sampledata[i][begin:end], label=legendthings[i], linewidth='0.7', color=colors[i], marker=".")

ax.plot(x_axis[begin:end], average[begin:end], label='Average', color='r', linestyle='--')

#ax2 = plt.axes([0,0,1,1])
#ip = InsetPosition(ax, [0.4,0.2,0.5,0.5])
#ax2.set_axes_locator(ip)

#for i in range(5):
#    ax.plot(x_axis, sampledata[i], label=legendthings[i], linewidth='0.7', color=colors[i], marker=".")

#ax2.plot(x_axis, average, label='Average', color='r', linestyle='--')

#for value in highest_values:
 #  plt.axhline(value, c='r',linestyle='--')
 #   plt.text(end+130,value+2,str(int(value)))
    #for marker in markers:
#    plt.axvline(marker,0,250,c='grey',linestyle='--')
fig.subplots_adjust(left=0.2)

plt.title("Amount of samples per second")
plt.xlabel('second', fontsize=12)
plt.ylabel('samples', fontsize=12)
plt.grid(True)

plt.legend()
#plt.show()

plt.savefig('sps.png')

