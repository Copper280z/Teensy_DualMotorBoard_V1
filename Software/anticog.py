# -*- coding: utf-8 -*-
"""
Created on Sun Feb  25 14:43:22 2024

@author: bob 
"""
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning) 
warnings.filterwarnings("ignore", category=FutureWarning) 
import numpy as np
import pandas as pd
import TeensyMotorControl as tc
import time
from matplotlib import pyplot as plt
from scipy import signal, interpolate
plt.rcParams["figure.dpi"] = 200 #Use this with 4k screens



def getVelocity(df):
    # dt = pd.Series(df.index).diff().fillna(method='bfill').fillna(method='ffill')
    dt = pd.Series(df.index).fillna(method='bfill').fillna(method='ffill')
    dt = np.gradient(dt.to_numpy())

    # velocity = df['motor.state1.ymech'].diff().fillna(method='bfill')
    velocity = np.gradient(df['motor.state1.ymech'].fillna(method='bfill').to_numpy())
    # velocity = velocity.rolling(50, center=True).mean()
    velocity = np.nan_to_num(np.squeeze(velocity))/dt
    
    plt.figure()
    plt.title('velocity')
    plt.plot(pd.Series(df.index),velocity)
    plt.show()
    
    return velocity

def getAcceleration(df,velocity):
    # dt = pd.Series(df.index).diff().fillna(method='bfill').fillna(method='ffill')
    dt = pd.Series(df.index).fillna(method='bfill').fillna(method='ffill')
    dt = np.gradient(dt.to_numpy())
    
    # dt = dt.to_numpy()
    accel = np.gradient(velocity)/dt

    return accel

def init_motor():
    m.CL_cur( 0 )
    motor.conf1.ridethewave = 1
    time.sleep(0.5)
       
    motor.conf1.commutationoffset = 0
       
    motor.state1.Valpha_offset = 6
       
    time.sleep(1)
       
    offset1 = motor.state1.thetaPark_enc
       
    motor.state1.Valpha_offset = 0
       
    motor.conf1.commutationoffset = -offset1
    print(f'Commutation offset: {-offset1}')
       
    Lq = 3698e-6
    Ld = 2376e-6
       
    R = 6.87
    m.setpar('motor.conf1.Lambda_m', 0.019)
    m.setpar('motor.conf1.N_pp',  11)
    m.setpar('motor.conf1.Lq', Lq)
    m.setpar('motor.conf1.Ld', Ld)
    m.setpar('motor.state1.R', R)
       
       
    m.CL_cur( 0.5e3 , 1)
    m.setpar('motor.conf1.anglechoice', 0)


def find_min(start, n_points, direction):
    Iq_test = direction*np.linspace(start,0,n_points)
    m.setpar('s1.Iq_offset_SP', direction*Imax)
    time.sleep(0.05)
    m.setpar('s1.Iq_offset_SP', direction*start)
    time.sleep(0.5)
    for I in Iq_test:
        Ic_min = I
        m.setpar('s1.Iq_offset_SP', Ic_min)
        print(f'Iq: {I:.3f}')
        time.sleep(1)
        df = m.trace(1)
        vel_arr = getVelocity(df)
        vel = np.abs(np.mean(vel_arr))
        
        print(f'velocity: {vel:.3f}')
        if vel < vel_threshold:
            #maybe check a couple of times to make sure it's actually stopped?
            print('Motor Stopped!')
            print(f'Ic_min = {Ic_min}')
            break
    Ic_min *= 1.05
    return Ic_min

def measure(Ic):
    if Ic > 0:
        direction = 1
    else:
        direction = -1
        
    m.setpar('s1.Iq_offset_SP', 0)
    time.sleep(0.2)
    m.setpar('s1.Iq_offset_SP', direction*Imax)
    time.sleep(0.2)
    m.setpar('s1.Iq_offset_SP', Ic)
    
    time.sleep(3)
    df = m.trace(4.0)
    velocity = getVelocity(df)
    df['angle'] = df['motor.state1.ymech'] % (2*np.pi)
    theta = np.squeeze(df['angle'].to_numpy())
    idx = np.argsort(theta)
    theta = theta[idx]
    velocity=velocity[idx]#[100:-100]
    
    accel = getAcceleration(df,velocity)
    
    angle_bins = np.linspace(0,2*np.pi,4000)
    
    digitized = np.digitize(theta, angle_bins)
    velocity_bin_means = [velocity[digitized == i].mean() for i in range(0, len(angle_bins))]
    velocity_bin_means = np.nan_to_num(velocity_bin_means,nan=np.nanmean(velocity_bin_means))
    
    vf = np.fft.rfft(velocity_bin_means)
    velocity_fit = np.fft.irfft(vf[:fft_bins], len(angle_bins))
    
    accel_bin_means = [accel[digitized == i].mean() for i in range(0, len(angle_bins))]
    accel_bin_means = np.nan_to_num(accel_bin_means,nan=np.nanmean(accel_bin_means))
    
    af = np.fft.rfft(accel_bin_means)
    accel_fit = np.fft.irfft(af[:fft_bins], len(angle_bins))

    ret = {'raw_angle':theta,
           'raw_velocity':velocity,
           'angle':angle_bins,
           'velocity':velocity_bin_means,
           'filtered velocity':velocity_fit,
           'accel':accel,
           'filtered_accel': accel_fit}
    return ret

m = tc.Motor(  )
motor = tc.MotorVariables( m )
m.setpar('motor.conf1.cog_ff_gain', 0)

init_motor()

m.setTrace(['motor.state1.thetaPark', 
            'motor.state1.encoderPos1',
            'motor.state1.ymech',
            'motor.state.curtime', 
            'motor.state1.Vd', 
            'motor.state1.Vq', 
            'motor.state1.Iq_meas', 
            'motor.state1.Id_meas',
            'motor.state1.Iq_SP',
            'motor.state1.Iq_offset_SP'])
Imax = 0.5
Imin_start = 0.1
m.setpar('s1.Iq_offset_SP', 0)
fft_bins = 240
n_points = 20
Iq_test = np.linspace(0.05,0.5,20)


vels = []
for i in range(3):
    df = m.trace(0.5)
    df['angle'] = (df['motor.state1.ymech'])
    stationary_velocity = np.abs(np.mean(getVelocity(df)))
    vels.append(stationary_velocity)
vel_threshold = np.maximum(0.02,9*np.std(vels))


for I in Iq_test:
    Ic_max = I
    m.setpar('motor.state1.Iq_offset_SP',Ic_max)
    print(f'Iq: {I:.3f}')
    time.sleep(0.5)
    df = m.trace(0.1)
    vel = np.mean(getVelocity(df))
    
    if vel > vel_threshold:
        #maybe check a couple of times to make sure it's actually spinning?
        print('Motor moving!')
        print(f'Ic_max = {Ic_max}')
        break

m.setpar('motor.state1.Iq_offset_SP',0)
    
Imax = Ic_max
# %%
fft_bins = 75
Icog = np.zeros(4000)
print('searching for Ic_min')

J =  0.00002*2.2
Km = 0.019

Ic_min = [0.028]
# Ic_min = [0.021]
m.setpar('motor.state1.Iq_offset_SP',Ic_min[0])

# Ic_min_n = [-0.03013158]
# Ic_min = [find_min(Imin_start, n_points, 1)]
# Ic_min_n = [find_min(Imin_start, n_points, -1)]
# Ic_min = Ic_min + Ic_max/(n_points)

mp = []
for I in Ic_min:
    mp.append(measure(I))

mn = []
for I in Ic_min:
    mn.append(measure(-I))
    
m.setpar('motor.state1.Iq_offset_SP',Ic_min[0])

plt.figure()
plt.title('accel')

for meas_p in mp:
    plt.plot(meas_p['angle'], J*meas_p['accel']/Km)
plt.show()
meas_p = mp[0]
meas_n = mn[0]

mean_accel = np.mean([meas_p['accel'], -meas_n['accel']], axis=0)

plt.figure()
plt.title('Velocity at minimum positive Iq')
plt.plot(meas_p['angle'], meas_p['velocity'], label="velocity")
plt.plot(meas_p['angle'], meas_p['filtered velocity'],'r', label="filtered velocity")
plt.plot(meas_n['angle'], -meas_n['filtered velocity'],'g', label="-filtered velocity")
plt.plot(meas_p['angle'], meas_p['accel'],'m', label="accel")
plt.plot(meas_p['angle'], -meas_n['accel'],'c', label="-accel")
plt.plot(meas_p['angle'], mean_accel,'b', label="mean accel")
plt.legend()
plt.show()

Icog = J*mean_accel/Km
plt.figure()
plt.title('Cogging correction current')
plt.plot(Icog)
plt.plot(Icog,'r.')
plt.show()


#%%
if not np.all(np.isnan(Icog)):

    for i,Ic in enumerate(np.roll(Icog,0)):
        m.setparpart('motor.conf1.cog_torque_ff', Ic, i)
    m.setpar('motor.conf1.cog_ff_gain', -1)
