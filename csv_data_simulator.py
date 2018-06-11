import numpy as np
from matplotlib import pyplot as plt
from matplotlib import interactive
import glob, os
import sys
import time



directory = "D:\\Dropbox\\Kroova LLC\\P105 - Moth Control System\\Testing Data\\Ultra Sonic Vs Wand 5-26-18\\Analysis"
os.chdir(directory)
ii = 0
for file in glob.glob("*.csv"):
        print(file)
        fname = directory+"\\"+file
        print(fname)
        data = np.genfromtxt(fname,delimiter=",")
        print("Time: ",data[np.shape(data)[0]-1,0]-data[0,0])


        fig, ax1 = plt.subplots(figsize=(8, 4))
        plt.title(file)
        ax1.plot(data[:,1], 'b-')

        ax2 = ax1.twinx()
        ax2.plot(data[:,2], 'r.')

        plt.pause(0.1)
        
        input("Press Enter to continue...")
        plt.close()	
        ii += 2

print(np.shape(data))
plt.close('all')


