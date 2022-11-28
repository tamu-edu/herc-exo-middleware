from turtle import write_docstringdict
from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
from ActPackMan import ActPackMan
import numpy as np
import time
import csv

def calibrate(dev, dt=0.001):
    print("Press CTRL-C to finish.")
    dev.set_current_gains()
    loop = SoftRealtimeLoop(dt = 1/277, report=True, fade=0.01)
    f = open("/home/pi/MBLUE/data/ankle_calibration%s.csv"%timestamp, 'w', encoding='UTF8', newline='')
    writer = csv.writer(f)
    for i, t in enumerate(loop):
        dev.update()
        dev.τ= 0.2
        ank_ang = dev.act_pack.ank_ang * 2*np.pi / pow(2,14) * 180/np.pi # degrees 
        mot_ang = dev.get_motor_angle_radians()
        writer.writerow([ank_ang, mot_ang])
        # if i%100==0:
        #     print('Ankle angle: ', ank_ang)

        

def main():
    with ActPackMan('/dev/ttyActPackA', gear_ratio=1, updateFreq=1000) as dev:
        calibrate(dev)
    print("calibration")

if __name__ == '__main__':
    main()
