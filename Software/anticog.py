# -*- coding: utf-8 -*-
"""
Created on Sun Feb  25 14:43:22 2024

@author: bob
"""
import numpy as np
import pandas as pd
import TeensyMotorControl as tc
import time
from matplotlib import pyplot as plt
from scipy import signal, interpolate

def getVelocity(df):
    dt = pd.Series(df.index).diff().fillna(method='bfill').fillna(method='ffill')
    dt = dt.to_numpy()

    velocity = df.filter(regex='thetaPark').diff().fillna(method='bfill')
    velocity = velocity.rolling(25).median()
    velocity = np.nan_to_num(np.squeeze(velocity.to_numpy()))/dt/motor.conf1.N_pp
    
    return velocity

def getAcceleration(df,velocity):
    dt = pd.Series(df.index).diff().fillna(method='bfill').fillna(method='ffill')
    dt = dt.to_numpy()
    accel = np.diff(velocity)/dt[1:]

    return accel

m = tc.Motor(  )
motor = tc.MotorVariables( m )


m.setTrace(['motor.state1.thetaPark', 
            'motor.state.curtime', 
            'motor.state1.Vd', 
            'motor.state1.Vq', 
            'motor.state1.Iq_meas', 
            'motor.state1.Id_meas',
            'motor.state1.Iq_offset_SP'])

m.setpar('s1.Iq_offset_SP', 0)
fft_bins = 15
n_points = 100
Iq_test = np.linspace(0,1.5,n_points)

vels = []
for i in range(10):
    df = m.trace(0.1)
    stationary_velocity = np.mean(getVelocity(df))
    vels.append(stationary_velocity)
vel_threshold = 6*np.std(vels)


for I in Iq_test:
    Ic_max = I
    m.setpar('s1.Iq_offset_SP', Ic_max)
    print(f'Iq: {I:.3f}')
    time.sleep(0.2)
    df = m.trace(0.1)
    vel = np.mean(getVelocity(df))
    
    if vel > vel_threshold:
        #maybe check a couple of times to make sure it's actually spinning?
        print('Motor moving!')
        print(f'Ic_max = {Ic_max}')
        break

print('searching for Ic_min')
Iq_test = np.linspace(Ic_max,0,n_points)
for I in Iq_test:
    Ic_min = I
    m.setpar('s1.Iq_offset_SP', Ic_min)
    print(f'Iq: {I:.3f}')
    time.sleep(1)
    df = m.trace(0.1)
    vel = np.mean(getVelocity(df))
    
    if vel < vel_threshold:
        #maybe check a couple of times to make sure it's actually stopped?
        print('Motor Stopped!')
        print(f'Ic_min = {Ic_min}')
        break

Ic_min = Ic_min + Ic_max/(n_points)

m.setpar('s1.Iq_offset_SP', Ic_max)
time.sleep(0.1)
m.setpar('s1.Iq_offset_SP', Ic_min)

time.sleep(2)

df = m.trace(1.0)
velocity = getVelocity(df)
theta = np.squeeze(df.filter(regex='theta').to_numpy())

idx = np.argsort(theta)
theta = theta[idx]
velocity=velocity[idx]

angle_bins = np.linspace(0,2*np.pi,2000)

digitized = np.digitize(theta, angle_bins)
bin_means = [velocity[digitized == i].mean() for i in range(1, len(angle_bins))]
angle_bins = angle_bins[1:]

f = np.fft.rfft(bin_means)
fit = np.fft.irfft(f[:fft_bins], 2*len(f)-1)

N=len(f)
k2 = np.zeros(N)
if ((N%2)==0):
    #-even number                                                                                   
    for i in range(1,N//2):
        k2[i]=i
        k2[N-i]=-i
else:
    #-odd number                                                                                    
    for i in range(1,(N-1)//2):
        k2[i]=i
        k2[N-i]=-i

dfit = np.fft.ifft((1j*k2*f)[:fft_bins],2*len(f)-1)

plt.figure()
plt.title('Velocity at minimum Iq')
plt.plot(theta, velocity,'.')
plt.plot(angle_bins, bin_means)
plt.plot(angle_bins,fit,'r')
plt.plot(angle_bins,dfit,'m')
plt.show()


Icog = Ic_min*dfit/np.max(np.abs(dfit))
