#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 15 15:25:45 2024

@author: bob
"""

import matplotlib.pyplot as plt
import pandas as pd
import struct
import numpy as np
import time
import control as ct
import TeensyMotorControl as tc
import pyqtgraph as pg

# For graph and tree:
from PyQt5.QtWidgets import QApplication, QGridLayout, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QLineEdit, QSlider, QCompleter
from PyQt5.QtCore import QObject, pyqtSignal, QEvent
from PyQt5.QtCore import Qt
import struct
from pyqtgraph.parametertree import Parameter, ParameterTree
from pyqtgraph.Qt import QtCore, QtWidgets
from pyqtgraph.widgets.PlotWidget import PlotWidget

def align_yaxis(ax1, ax2):
    y_lims = np.array([ax.get_ylim() for ax in [ax1, ax2]])

    # force 0 to appear on both axes, comment if don't need
    y_lims[:, 0] = y_lims[:, 0].clip(None, 0)
    y_lims[:, 1] = y_lims[:, 1].clip(0, None)

    # normalize both axes
    y_mags = (y_lims[:,1] - y_lims[:,0]).reshape(len(y_lims),1)
    y_lims_normalized = y_lims / y_mags

    # find combined range
    y_new_lims_normalized = np.array([np.min(y_lims_normalized), np.max(y_lims_normalized)])

    # denormalize combined range to get new axes
    new_lim1, new_lim2 = y_new_lims_normalized * y_mags
    ax1.set_ylim(new_lim1)
    ax2.set_ylim(new_lim2)

pi = np.pi
# plt.rcParams["figure.dpi"] = 200 #Use this with 4k screens
plt.rcParams.update({"axes.grid": True})
plt.rcParams.update({"legend.loc": 'upper left'})
pg.setConfigOption('background', 'k')
pg.setConfigOption('foreground', 'w')

m = tc.Motor(  )
motor = tc.MotorVariables( m )

z = ct.TransferFunction( [1, 0] , [1] , float(m.Ts))

# Run code till here, and you will have the m and motor objects to work with 
m.setpar('motor.state1.muziek_gain', .0)
# motor.conf1.enccountperrev = 4096
# motor.conf1.enc2rad = (2*np.pi)/motor.conf1.enccountperrev

#%% Commutate with encoder
m.CL_cur( 0 )
motor.conf1.ridethewave = 1
time.sleep(0.5)

motor.conf1.commutationoffset = 0

motor.state1.Valpha_offset = 2

time.sleep(1)

offset1 = motor.state1.thetaPark_enc

motor.state1.Valpha_offset = 0

motor.conf1.commutationoffset = -offset1
print(f'Commutation offset: {-offset1}')
# %%
# 0.4A of d-axis current
# Lq = 2000e-6
# Ld = 2900e-6

# 0.2A of d-axis current
Lq = 3103e-6
Ld = 2543e-6

# 0 current bias
# Lq = 3000e-6
# Ld = 2500e-6

R = 10.8
m.setpar('motor.conf1.Lambda_m', 0.008)
m.setpar('motor.conf1.N_pp',  7)
m.setpar('motor.conf1.Lq', Lq)
m.setpar('motor.conf1.Ld', Ld)
m.setpar('motor.state1.R', R)

m.CL_cur( 0.5e3 , 1)

#%%
m.setpar('motor.state1.Iq_offset_SP',0.1)
time.sleep(1)
m.setpar('motor.state1.Iq_offset_SP',0.0)


#%%
signals = [ 's1.encoderPos1','s1.thetaPark_enc', 's1.BEMFa', 's1.BEMFb']
m.setTrace(signals)
m.setpar('motor.conf1.anglechoice', 1)
m.setpar('motor.state1.Iq_offset_SP',0.2)

df = m.trace(1)
m.setpar('motor.state1.Iq_offset_SP',0.0)

ax1 = df.filter(regex='encoder').plot()
ax2 = ax1.twinx()
df.filter(regex='thetaPark').plot(ax=ax2,color='orange')

ax1 = df.filter(regex='encoder').plot()
ax2 = ax1.twinx()
df.filter(regex='BEMF').plot()

m.setpar('motor.conf1.anglechoice', 101)

#%% hfi on
signals = [ 's1.delta_iq','s1.delta_id', 's1.hfi_dir', 's1.thetaPark_enc', 's1.Iq_meas', 's1.hfi_V', 's1.hfi_curangleest']
m.setTrace(signals)

m.setpar('s1.hfi_use_lowpass', 1)
m.setpar('s1.hfi_method', 5)

# Ki = 0.01*2*pi
Ki = 750*2*pi
hfi_v = 18

m.setpar('s1.hfi_maxvel', 1e6)
m.setpar('s1.hfi_gain', Ki)
# m.setpar('s1.hfi_gain_int2', 0.001*2*pi) # 5*2*pi
m.setpar('s1.hfi_gain_int2', 5*2*pi) # 5*2*pi
m.setpar('s1.hfi_V', hfi_v)
m.setpar('s1.hfi_dir_int',0)
m.setpar('s1.hfi_contout',0)
m.setpar('s1.hfi_on', 1)
m.setpar('c1.anglechoice', 3)

# m.setpar('motor.conf1.Lq', 3270e-6)
# m.setpar('motor.conf1.Ld', 2900e-6)
# time.sleep(0.5)
# Lq = 2700e-6
# Ld = 1900e-6
# m.setpar('motor.conf1.Lq', Lq)
# m.setpar('motor.conf1.Ld', Ld)
m.setpar('motor.state1.Id_offset_SP',0.2)
time.sleep(1.0)

m.setpar('motor.state1.Iq_offset_SP',0.3)

df = m.trace(1)

m.setpar('motor.state1.Id_offset_SP',0.0)
m.setpar('motor.state1.Iq_offset_SP',0.0)
# m.setpar('motor.conf1.Lq', 3270e-6)
# m.setpar('motor.conf1.Ld', 2900e-6)
m.setpar('s1.hfi_on', 0)
m.setpar('s1.hfi_method', 5)
m.setpar('motor.conf1.anglechoice', 0)


df.filter(regex='delta').rolling(100).mean().plot()

ax1 = df.filter(regex='dir').plot()
ax1.plot(df.filter(regex='theta'))
# df.filter(regex='thetaPark_enc').plot(ax=ax1,color='orange')
# diff_df = df.apply(np.diff)
# diff_df.index = df.index[:-1]
# ax2 = ax1.twinx()
# diff = diff_df.filter(regex='dir').rolling(100).mean()
# diff[diff>4] = 0
# diff[diff<-4] = 0
# ax2.plot(diff, color='red')



# df_fft = m.getFFTdf(df,1, 1000)
# f = df_fft.index.values
# plt.figure()
# m.bode(df_fft.filter(regex='Iq_meas'),f)
# plt.semilogx(20*(np.log10(np.abs(np.fft.fft(df.filter(regex='Iq_meas'))))))

d_id = df.filter(regex='delta_id').to_numpy()
d_iq = df.filter(regex='delta_iq').to_numpy()
t_hfi = df.filter(regex='dir').to_numpy()
t_enc = df.filter(regex='theta').to_numpy()
v_hfi = df.filter(regex='theta').to_numpy()
t_err = t_enc-t_hfi

hfi_est_err = 0.25*np.arctan2(-d_iq,d_id-0.5*v_hfi*m.Ts*(1/Lq+1/Ld))

hfi_est_err_filt = np.convolve(hfi_est_err.ravel(), np.ones((500,)))

plt.figure()
# plt.plot(hfi_est_err)
# plt.plot(hfi_est_err_filt)
plt.plot(df.filter(regex='hfi_curangleest').rolling(50).mean().to_numpy())
ax1=plt.gca()
ax2=ax1.twinx()
ax2.plot(t_err,'r.')
align_yaxis(ax1, ax2)
# plt.legend(['estimated error', 'true error'])
plt.show()

#%%  
m.setpar('s1.hfi_use_lowpass', 1)
m.setpar('s1.hfi_method', 5)

# Ki = 0.01*2*pi
Ki = 750*2*pi
hfi_v = 18

m.setpar('s1.hfi_maxvel', 1e6)
m.setpar('s1.hfi_gain', Ki)
# m.setpar('s1.hfi_gain_int2', 0.001*2*pi) # 5*2*pi
m.setpar('s1.hfi_gain_int2', 5*2*pi) # 5*2*pi
m.setpar('s1.hfi_V', hfi_v)
m.setpar('s1.hfi_dir_int',0)
m.setpar('s1.hfi_contout',0)
m.setpar('s1.hfi_on', 1)
m.setpar('c1.anglechoice', 3)
m.setpar('motor.state1.Id_offset_SP',0.2)

m.setpar( 's1.hfi_useforfeedback' , 1)
m.setpar( 'motor.conf1.maxerror' , 1e6)
# m.setpar( 'motor.conf1.maxerror' , 1e6)

# m.CL( 1, 1, J=0.00021)
m.CL( 7, 1, J=0.0000021)
#%%
m.setpar( 'motor.conf1.Command' , 2)

#%%  
m.prepSP( 360/360*2*pi , 200 , 2000 ,250000)
N = 1
m.setpar('motor.state1.SPdir' , 1)
m.setpar('motor.state1.spNgo' , N)

while (m.getsig('motor.state1.spNgo') > 0 or m.getsig('motor.state1.REFstatus') > 0 ):
    bla = 1;
m.setpar('motor.state1.SPdir' , 0)
m.setpar('motor.state1.spNgo' , N)
# %% Current loop axis 1  
if 1:
    m.setpar('s1.hfi_method', 0)
    m.setpar('motor.conf1.anglechoice', 0)
    
    NdownsamplePRBS = 3
    N = 30*NdownsamplePRBS*2047
    signals = ['motor.state1.Id_meas', 'motor.state1.Iq_meas',
               'motor.state1.dist', 'motor.state1.Vq', 'motor.state1.Vd']
    m.setTrace(signals )
    
    
    gain = 5
    m.setpar('motor.state1.Id_offset_SP',0.0)

    # m.setpar('motor.state1.Valpha_offset', 0)
    # time.sleep(0.5)
    m.setpar('motor.state1.Vq_distgain', 1)
    m.setpar('motor.state1.Vd_distgain', 1)
    m.setpar('motor.state1.Iq_distgain', 0)
    m.setpar('motor.state1.Id_distgain', 0)
    m.setpar('motor.state1.mechdistgain', 0)
    
    m.setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
    m.setpar('motor.state1.distval', gain)  # disturbance amplitude
    m.setpar('motor.state1.distoff', 0)  # disturbance offset
    df = m.trace(N * m.Ts)
    # m.setpar('motor.state1.Valpha_offset', 0)
    m.setpar('motor.state1.distval', 0)  # disturbance amplitude
    m.setpar('motor.state1.distoff', 0)  # disturbance offset
    m.setpar('motor.conf.NdownsamplePRBS', 1)  # Downsampling
    m.setpar('motor.state1.Id_offset_SP',0.0)

    dfout = m.getFFTdf(df, NdownsamplePRBS , 10*2047 )
    f = dfout.index.values
    Pd = dfout['motor.state1.Id_meas'].values / dfout['motor.state1.Vd'].values
    Pq = dfout['motor.state1.Iq_meas'].values / dfout['motor.state1.Vq'].values
    Sd = dfout['motor.state1.Vd'].values
    Sq = dfout['motor.state1.Vq'].values
    
    # %% Current loop plots
    plt.figure(1)
    m.bode( Pd , f, 'Measured D axis plant')
    m.bode( Pq , f, 'Measured Q axis plant')
    
    plt.figure(2)
    m.bode( 1 / Sd - 1 , f, 'Open loop D')
    m.bode( 1 / Sq - 1 , f, 'Open loop Q')
    
    # plt.figure(6)
    # m.bode( (1 / Sd - 1)/Pd , f, 'Controller D')
    # m.bode( (1 / Sq - 1)/Pq , f, 'Controller Q')
    
    
    # plt.figure(3)
    # m.nyquist( 1 / Sd - 1 , f, 'Open loop D')
    # m.nyquist( 1 / Sq - 1 , f, 'Open loop Q')
    
    Ld_arr = np.abs(1/(Pd * f * 2 * np.pi)) * 1e6
    Lq_arr = np.abs(1/(Pq * f * 2 * np.pi)) * 1e6
    
    plt.figure(4)
    plt.plot(
        f, Ld_arr)
    plt.grid()
    plt.xlim([1e3, 10e3])
    plt.ylim([ 100 , 5000])
    plt.title(f'Ld [uH]\nmean: {np.mean(Ld_arr[f>2000]):.3f}')
    
    
    plt.figure(5)
    plt.plot(
        f, Lq_arr)
    plt.grid()
    plt.xlim([1e3, 10e3])
    plt.ylim([ 100 , 5000])
    plt.title(f'Lq [uH]\nmean: {np.mean(Lq_arr[f>2000]):.3f}')
    
    plt.figure(7)
    plt.plot(
        f, (np.abs(1/(Pd * f * 2 * np.pi))) / np.abs(1/(Pq * f * 2 * np.pi)) )
    plt.grid()
    plt.xlim([1e3, 10e3])
    plt.ylim([ 0.5 , 1.5])
    plt.title('Ld/Lq ')

# %%
if 0:
#%%        
    # For Ld and Lq gain I would look at the ampltiude of the sine waves in your plot.   i_q(crosstalk_oninject) = (1/Lq - 1/Ld) * 0.5* V * Ts sin(2*theta_error) ;
    m.setpar('s1.hfi_method', 1)
    
    Ki = 1000*2*pi
    hfi_v = 15
    
    m.setpar('s1.hfi_maxvel', 1e6)
    m.setpar('s1.hfi_gain', Ki)
    m.setpar('s1.hfi_gain_int2', 5*2*pi)
    m.setpar('s1.hfi_V', hfi_v)
    m.setpar('s1.hfi_on', 1)
    
    m.setpar('motor.conf1.anglechoice', 101)
    m.setpar('motor.state1.Id_offset_SP',0.3)
    m.setpar('motor.state1.Iq_offset_SP',0)
    
    
    # m.setpar('motor.state1.thetaPark', 0)
    
    # vmax = 500  # ERPM
    # m.setpar('motor.conf1.anglechoice', 99)
    # m.setpar('motor.state1.i_vector_acc', 100000)
    # m.setpar('motor.state1.i_vector_radpers', vmax / 60 * 2*pi)
    
    signals = ['motor.state1.BEMFa', 'motor.state1.BEMFb', 'motor.state.sensBus', 'motor.state1.Iq_meas', 'motor.state1.Id_meas',  'motor.state1.Id_e', 'motor.state1.Iq_e', 's1.delta_id', 's1.delta_iq', 'motor.state1.thetaPark' , 'motor.state1.thetaPark_enc', 'motor.state.is_v7']
    m.setTrace(signals)
    
    df = m.trace(8)
    
    m.setpar('s1.hfi_on', 0)
    m.setpar('motor.conf1.anglechoice', 0)
    m.setpar('motor.state1.Id_offset_SP',0)
    m.setpar('motor.state1.Iq_offset_SP',0)
    
    df.filter(regex='delta').rolling(100).mean().plot()

# %% Debug HFI rotating vector
Lq = 3800e-6
Ld = 4800e-6
R = 10.8
m.setpar('motor.conf1.Lambda_m', 0.005405)
m.setpar('motor.conf1.N_pp',  7)
m.setpar('motor.conf1.Lq', Lq)
m.setpar('motor.conf1.Ld', Ld)
m.setpar('motor.state1.R', R)


m.setpar('s1.hfi_method', 1)
Ki = 1000*2*pi

m.setpar('s1.hfi_maxvel', 1e6)
m.setpar('s1.hfi_gain', Ki)
m.setpar('s1.hfi_gain_int2', 5*2*pi)
m.setpar('s1.hfi_V', 2)
m.setpar('s1.hfi_on', 1)

m.setpar('motor.conf1.anglechoice', 99)
m.setpar('motor.state1.i_vector_acc', 1e8)


signals = ['motor.state1.BEMFa', 'motor.state1.BEMFb', 'motor.state.sensBus', 'motor.state1.Iq_meas', 'motor.state1.Id_meas',
           'motor.state1.Id_e', 'motor.state1.Iq_e', 's1.delta_id', 's1.delta_iq', 'motor.state1.thetaPark']
m.setTrace(signals)
Ldest_uH = []
Lqest_uH = []

# for hfi_v in [ 1 , 2 , 3 , 4  ]:
for hfi_v in [ 3 , 6, 9, 12, 15 ]:
  m.setpar('s1.hfi_V', hfi_v)
  motor.state1.thetaPark = 0
  m.setpar('motor.state1.i_vector_radpers', 2*pi)
  df = m.trace(1)
  m.setpar('motor.state1.i_vector_radpers', 0*pi)
  
  # df.filter(regex='delta_i|thetaPark').rolling(100).mean().plot()
  
  plt.figure(1)
  (df['motor.state1.delta_iq'].rolling(100).mean()/m.Ts).plot()
  (df['motor.state1.delta_id'].rolling(100).mean()/m.Ts).plot()
  # df['motor.state1.thetaPark'].rolling(100).mean().plot()
  plt.ylabel('Delta current [A/s]')
  
  plt.figure(2)
  (df['motor.state1.delta_iq'].rolling(100).mean()/(hfi_v*m.Ts)).plot()
  (df['motor.state1.delta_id'].rolling(100).mean()/(hfi_v*m.Ts)).plot()
  # df['motor.state1.thetaPark'].rolling(100).mean().plot()
  plt.ylabel('Delta current [A/s per volt]')

  DID = np.fft.fft( df['motor.state1.delta_id']/ (hfi_v * m.Ts * len(df)) )

  avg = np.abs(DID[0])
  amp = np.abs(DID[2]*2) #Note: factor 2 used to go from dual sided FFT to single sides FFT.
  
  Ldest_uH.append(1e6/(avg + amp))
  Lqest_uH.append(1e6/(avg - amp))

m.setpar('s1.hfi_on', 0)