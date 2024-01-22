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


m.setpar('motor.state1.muziek_gain', 1)

# %%
Lq = 400e-6
Ld = 360e-6
R = 0.8
m.setpar('motor.conf1.Lambda_m', 0.005405)
m.setpar('motor.conf1.N_pp',  4)
m.setpar('motor.conf1.Lq', Lq)
m.setpar('motor.conf1.Ld', Ld)
m.setpar('motor.state1.R', R)

m.CL_cur( 2.5e3 , 1)

#%% hfi on
m.setpar('s1.hfi_use_lowpass', 1)
m.setpar('s1.hfi_method', 1)

Ki = 200*2*pi
hfi_v = 19

m.setpar('s1.hfi_maxvel', 1e6)
m.setpar('s1.hfi_gain', Ki)
m.setpar('s1.hfi_gain_int2', 7*2*pi) # 5*2*pi
m.setpar('s1.hfi_V', hfi_v)
m.setpar('s1.hfi_on', 1)
m.setpar('c1.anglechoice', 3)

#%%
signals = ['motor.state1.hfi_dir', 's1.delta_iq']
m.setTrace(signals)

df = m.trace(4)
df.rolling(10).mean().plot()


#%% hfi off
m.setpar('s1.hfi_on', 0)
m.setpar('s1.hfi_method', 1)
m.setpar('motor.conf1.anglechoice', 101)

# %% Current loop axis 1  
if 1:
    m.setpar('s1.hfi_method', 0)
    m.setpar('motor.conf1.anglechoice', 0)
    
    NdownsamplePRBS = 2
    N = 30*NdownsamplePRBS*2047
    signals = ['motor.state1.Id_meas', 'motor.state1.Iq_meas',
               'motor.state1.dist', 'motor.state1.Vq', 'motor.state1.Vd']
    m.setTrace(signals )
    
    
    gain = 0.5
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
    
    plt.figure(4)
    plt.plot(
        f, np.abs(1/(Pd * f * 2 * np.pi)) * 1e6)
    plt.grid()
    plt.xlim([1e3, 10e3])
    plt.ylim([ 100 , 500])
    plt.title('Ld [uH]')
    
    
    plt.figure(5)
    plt.plot(
        f, np.abs(1/(Pq * f * 2 * np.pi)) * 1e6)
    plt.grid()
    plt.xlim([1e3, 10e3])
    plt.ylim([ 100 , 500])
    plt.title('Lq [uH]')
    
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
    hfi_v = 19
    
    m.setpar('s1.hfi_maxvel', 1e6)
    m.setpar('s1.hfi_gain', Ki)
    m.setpar('s1.hfi_gain_int2', 5*2*pi)
    m.setpar('s1.hfi_V', hfi_v)
    m.setpar('s1.hfi_on', 1)
    
    m.setpar('motor.conf1.anglechoice', 101)
    m.setpar('motor.state1.Id_offset_SP',3)
    m.setpar('motor.state1.Iq_offset_SP',0)
    
    
    # m.setpar('motor.state1.thetaPark', 0)
    
    # vmax = 500  # ERPM
    # m.setpar('motor.conf1.anglechoice', 99)
    # m.setpar('motor.state1.i_vector_acc', 100000)
    # m.setpar('motor.state1.i_vector_radpers', vmax / 60 * 2*pi)
    
    signals = ['motor.state1.BEMFa', 'motor.state1.BEMFb', 'motor.state.sensBus', 'motor.state1.Iq_meas', 'motor.state1.Id_meas',  'motor.state1.Id_e', 'motor.state1.Iq_e', 's1.delta_id', 's1.delta_iq', 'motor.state1.thetaPark' , 'motor.state1.thetaPark_enc', 'motor.state.is_v7']
    m.setTrace(signals)
    
    df = m.trace(4)
    
    m.setpar('s1.hfi_on', 0)
    m.setpar('motor.conf1.anglechoice', 0)
    m.setpar('motor.state1.Id_offset_SP',0)
    m.setpar('motor.state1.Iq_offset_SP',0)
    
    df.filter(regex='delta').rolling(50).mean().plot()
