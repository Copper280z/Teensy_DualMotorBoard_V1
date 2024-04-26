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
    dt = pd.Series(df.index).diff().fillna(method='bfill').fillna(method='ffill')
    dt = dt.to_numpy()

    velocity = df['motor.state1.ymech'].diff().fillna(method='bfill')
    velocity = velocity.rolling(50, center=True).mean()
    velocity = np.nan_to_num(np.squeeze(velocity.to_numpy()))/dt
    
    plt.figure()
    plt.title('velocity')
    plt.plot(pd.Series(df.index),velocity)
    plt.show()
    
    return velocity

def getAcceleration(df,velocity):
    dt = pd.Series(df.index).diff().fillna(method='bfill').fillna(method='ffill')
    dt = dt.to_numpy()
    accel = np.diff(velocity)/dt[1:]

    return accel

def init_motor():
    Lq = 26.0e-6
    Ld = 19.5e-6

    R = 0.093
    m.setpar('motor.conf1.Lambda_m', 0.0013)
    # Ld = 2382e-6
    # Lq = 3342e-6
    # R=7.5
    
    m.setpar('motor.conf1.N_pp',  7)
    m.setpar('motor.conf1.Lq', Lq)
    m.setpar('motor.conf1.Ld', Ld)
    m.setpar('motor.state1.R', R)
    
    m.CL_cur( 0 )
    motor.conf1.ridethewave = 1
    time.sleep(0.5)
    
    motor.conf1.commutationoffset = 0
    
    motor.state1.Valpha_offset = 1.0
    
    time.sleep(1)
    
    offset1 = motor.state1.thetaPark_enc
    
    motor.state1.Valpha_offset = 0
    
    motor.conf1.commutationoffset = -offset1
    print(f'Commutation offset: {-offset1}')
    
    m.CL_cur( 1.0e3 , 1)
    # time.sleep(1)
    # m.setpar('s1.hfi_use_lowpass', 0)
    # m.setpar('s1.hfi_method', 1)
    
    # # Ki = 0.01*2*pi
    # Ki = 750*2*np.pi
    # hfi_v = 1.0
    
    # m.setpar('s1.hfi_maxvel', 1e6)
    # m.setpar('s1.hfi_gain', Ki)
    # # m.setpar('s1.hfi_gain_int2', 0.001*2*pi) # 5*2*pi
    # m.setpar('s1.hfi_gain_int2', 5*2*np.pi) # 5*2*pi
    # m.setpar('s1.hfi_V', hfi_v)
    # m.setpar('s1.hfi_dir_int',0)
    # m.setpar('s1.hfi_contout',0)
    # m.setpar('motor.state1.Id_offset_SP',0.1)
    # m.setpar('s1.hfi_on', 1)
    # m.setpar('c1.anglechoice', 0)
    # m.setpar('motor.state1.Id_offset_SP',0.0)
    # m.setpar( 's1.hfi_useforfeedback' , 1)
    # m.setpar( 'motor.conf1.maxerror' , 1e6)

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
    time.sleep(0.08)
    m.setpar('s1.Iq_offset_SP', Ic)
    
    time.sleep(3)
    df = m.trace(4.0)
    velocity = getVelocity(df)
    df['angle'] = df['motor.state1.ymech'] % (2*np.pi)
    theta = np.squeeze(df['angle'].to_numpy())
    idx = np.argsort(theta)
    theta = theta[idx]
    velocity=velocity[idx][100:-100]
    
    angle_bins = np.linspace(0,2*np.pi,4000)
    
    digitized = np.digitize(theta[100:-100], angle_bins)
    bin_means = [velocity[digitized == i].mean() for i in range(0, len(angle_bins))]
    bin_means = np.nan_to_num(bin_means,nan=np.nanmean(bin_means))
    # bin_means[0] = bin_means[1]
    
    f = np.fft.rfft(bin_means)
    fit = np.fft.irfft(f[:fft_bins], len(angle_bins))
    
    # N=len(f)
    # k2 = np.zeros(N)
    # if ((N%2)==0):
    #     #-even number                                                                                   
    #     for i in range(1,N//2):
    #         k2[i]=i
    #         k2[N-i]=-i
    # else:
    #     #-odd number                                                                                    
    #     for i in range(1,(N-1)//2):
    #         k2[i]=i
    #         k2[N-i]=-i
    
    # dfit = np.real(np.fft.ifft((1j*k2*f)[:fft_bins],2*len(f)-1))
    dfit = np.diff(fit)/np.diff(angle_bins)
    dfit = np.concatenate([np.array([dfit[0]]),dfit])

    ret = {'raw_angle':theta,
           'raw_velocity':velocity,
           'angle':angle_bins,
           'velocity':bin_means,
           'filtered velocity':fit,
           'accel':dfit}
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
Imax = 0.9
Imin_start = 0.2
m.setpar('s1.Iq_offset_SP', 0)
fft_bins = 240
n_points = 20
Iq_test = np.linspace(0.4,1.0,10)

vels = []
for i in range(3):
    df = m.trace(0.5)
    df['angle'] = (df['motor.state1.ymech'])
    stationary_velocity = np.abs(np.mean(getVelocity(df)))
    vels.append(stationary_velocity)
vel_threshold = np.maximum(0.02,9*np.std(vels))


# for I in Iq_test:
#     Ic_max = I
#     m.setpar('s1.Iq_offset_SP', Ic_max)
#     print(f'Iq: {I:.3f}')
#     time.sleep(0.2)
#     df = m.trace(0.1)
#     vel = np.mean(getVelocity(df))
    
#     if vel > vel_threshold:
#         #maybe check a couple of times to make sure it's actually spinning?
#         print('Motor moving!')
#         print(f'Ic_max = {Ic_max}')
#         break
    
# Imax = Ic_max
# %%
fft_bins = 200
Icog = np.zeros(4000)
print('searching for Ic_min')

J =  0.000009
Km = 0.0013

Ic_min = [0.145]
Ic_min_n = -0.145
# Ic_min = find_min(Imin_start, n_points, 1)
# Ic_min_n = find_min(Imin_start, n_points, -1)
# Ic_min = Ic_min + Ic_max/(n_points)
mp = []
for I in Ic_min:
    mp.append(measure(I))
m.setpar('s1.Iq_offset_SP', Ic_min[0])

plt.figure()
plt.title('accel')

for meas_p in mp:
    plt.plot(meas_p['angle'], J*meas_p['accel']/Km)
plt.show()
meas_p = mp[0]

plt.figure()
plt.title('Velocity at minimum positive Iq')
plt.plot(meas_p['angle'], meas_p['velocity'])
plt.plot(meas_p['angle'], meas_p['filtered velocity'],'r')
plt.plot(meas_p['angle'], meas_p['accel'],'m')
plt.show()

Icog = J*meas_p['accel']/Km
plt.figure()
plt.title('Cogging correction current')
plt.plot(Icog)
plt.plot(Icog,'r.')
plt.show()


#%%
if not np.all(np.isnan(Icog)):

    for i,Ic in enumerate(np.roll(Icog,0)):
        m.setparpart('motor.conf1.cog_torque_ff', Ic, i)
    m.setpar('motor.conf1.cog_ff_gain', -0.9)
