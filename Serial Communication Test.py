import serial
import time
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import sys
from PIL import Image
from scipy.ndimage import rotate

ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
ser.reset_input_buffer()
ser.flush()

counter = 0
grid = [0 for i in range(20)]

while True:
    ser.write(b"\n")
    line = ser.readline().decode('utf-8').rstrip()
    line = line.replace(" ", "")
    row = list(line)
    grid[counter] = row
    counter+=1
    print("Collected data from row: ", counter)
    
    if counter >= 20:
        print("Exiting loop")
        break


df = pd.DataFrame(grid)
df = df[df.columns].astype(int)

# Setting the indexes to start from 1.
df.index = df.index + 1
df.columns += 1

svm = sns.heatmap(df.T)
svm.invert_yaxis()
svm.set_xticklabels(svm.get_xticklabels(), rotation=180)


figure = svm.get_figure()
figure.savefig("heatmap.png", dpi = 1000)
np.savetxt(r'heatmap.txt', df.values, fmt='%d')
