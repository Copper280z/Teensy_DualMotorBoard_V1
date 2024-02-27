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
m.setpar('motor.state1.muziek_gain', 0.0)
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
# Lq = 3103e-6
# Ld = 2543e-6
Lq = 2950e-6
Ld = 2150e-6
# 0 current bias
# Lq = 3100e-6
# Ld = 2400e-6

R = 10.8
m.setpar('motor.conf1.Lambda_m', 0.008)
m.setpar('motor.conf1.N_pp',  7)
m.setpar('motor.conf1.Lq', Lq)
m.setpar('motor.conf1.Ld', Ld)
m.setpar('motor.state1.R', R)

m.CL_cur( 0.5e3 , 1)

#%%  
m.setpar('s1.hfi_use_lowpass', 1)
m.setpar('s1.hfi_method', 6)

m.setpar('motor.conf1.Lq_Iq_m', -444e-6)
m.setpar('motor.conf1.Ld_Id_m', -1222e-6)
m.setpar('motor.conf1.Lq_Iq_b', Lq)
m.setpar('motor.conf1.Ld_Id_b', Ld)
# Ki = 0.01*2*pi
Ki = 750*2*pi
hfi_v = 19

m.setpar('s1.hfi_maxvel', 1e6)
m.setpar('s1.hfi_gain', Ki)
# m.setpar('s1.hfi_gain_int2', 0.001*2*pi) # 5*2*pi
m.setpar('s1.hfi_gain_int2', 5*2*pi) # 5*2*pi
m.setpar('s1.hfi_V', hfi_v)
m.setpar('s1.hfi_dir_int',0)
m.setpar('s1.hfi_contout',0)
m.setpar('s1.hfi_on', 1)
m.setpar('c1.anglechoice', 3)
m.setpar('motor.state1.Id_offset_SP',0.1)

m.setpar( 's1.hfi_useforfeedback' , 1)
# m.setpar( 'motor.conf1.maxerror' , 1e6)
time.sleep(0.5)

# m.CL( 1, 1, J=0.00021)
m.CL( 7, 1, J=0.0000025)
m.setpar( 'motor.conf1.maxerror' , 2)
time.sleep(0.5)

# #%%
# m.setTrace([ 'motor.state1.rmech' , 'motor.state1.ymech' , 'motor.state1.emech' ,
#             'motor.state1.Vd', 'motor.state1.Vq', 'motor.state1.Iq_SP',
#             'motor.state1.Iq_meas', 'motor.state1.Id_meas', 'motor.state1.hfi_abs_pos', 
#             'motor.state1.hfi_curangleest','motor.state1.mechcontout' ,'motor.state1.T_FF_acc' ,
#             'motor.state1.T_FF_vel','motor.state.sensBus', 'motor.state1.Ld_fit', 'motor.state1.Lq_fit'])

# df = m.trace(3)

# df.filter(regex='_fit').plot()


#%%  
# m.setTrace( ['motor.state1.rmech' , 'motor.state1.vel', 'motor.state1.emech',  'motor.state1.ymech','motor.state1.Iq_SP'])

m.prepSP( 360/360*2*pi , 25 , 2000 ,500000)
N = 1
# df = m.tracebg()
m.setpar('motor.state1.SPdir' , 1)
m.setpar('motor.state1.spNgo' , N)
while (m.getsig('motor.state1.spNgo') > 0 or m.getsig('motor.state1.REFstatus') > 0 ):
    bla = 1;
m.setpar('motor.state1.SPdir' , 0)
m.setpar('motor.state1.spNgo' , N)
# df = m.stoptracegetdata()
# df.plot()
time.sleep(0.1)

if 0:
    m.setpar( 'motor.conf1.Command' , 2)
    
#%% Relative setpoints with trace
# time.sleep(10)

v = 500
a = 9000
jerk = 2500000
# a = 2000 #Only motor 1

motor.state1.Jload = 0.0000033

m.setpar('motor.state1.velFF' , 0.000020)
# m.setpar('motor.state2.velFF' , 0.00055)

m.setTrace([ 'motor.state1.rmech' , 'motor.state1.ymech' , 'motor.state1.emech' ,
            'motor.state1.Vd', 'motor.state1.Vq', 'motor.state1.Iq_SP',
            'motor.state1.Iq_meas', 'motor.state1.Id_meas', 'motor.state1.hfi_abs_pos', 
            'motor.state1.hfi_curangleest','motor.state1.mechcontout' ,'motor.state1.T_FF_acc' ,
            'motor.state1.T_FF_vel','motor.state.sensBus', 'motor.state1.Ld_fit', 
            'motor.state1.Lq_fit','motor.state1.thetaPark_enc', 'motor.state1.hfi_dir'])
m.tracebg(  )

# time.sleep(0.1)

m.rel( [ 360 , 360 ] , vel=v , acc = a, jerk = jerk)
time.sleep(0.1)
m.rel( [ -360 , -360] , vel=v , acc = a, jerk = jerk)
time.sleep(0.1)
m.rel( [ 360 , 360 ] , vel=v , acc = a, jerk = jerk)
time.sleep(0.1)
m.rel( [ -360 , -360 ] , vel=v , acc = a, jerk = jerk)
time.sleep(0.1)
m.rel( [ 360 , 360 ] , vel=v , acc = a, jerk = jerk)
time.sleep(0.1)
m.rel( [ -360 , -360] , vel=v , acc = a, jerk = jerk)
time.sleep(0.1)
m.rel( [ 360 , 360 ] , vel=v , acc = a, jerk = jerk)
time.sleep(0.1)
m.rel( [ -360 , -360 ] , vel=v , acc = a, jerk = jerk)
time.sleep(0.1)
m.rel( [ 360 , 360 ] , vel=v , acc = a, jerk = jerk)
time.sleep(0.1)
m.rel( [ -360 , -360] , vel=v , acc = a, jerk = jerk)
time.sleep(0.1)
m.rel( [ 360 , 360 ] , vel=v , acc = a, jerk = jerk)
time.sleep(0.1)
m.rel( [ -360 , -360 ] , vel=v , acc = a, jerk = jerk)
time.sleep(0.1)

# time.sleep(0.1)

df = m.stoptracegetdata()

# df.filter(regex='sens').plot()


df.filter(regex='state1..me|state1.me|state1.Iq').plot()

fig0, ax0 = plt.subplots()
ax1 = ax0.twinx()
df.filter(regex='state1.Iq|state1.Id').rolling(10).mean().plot(ax=ax0)
df.filter(regex='_fit').rolling(50).mean().plot(secondary_y=True,ax=ax1,style='--')



df.filter(regex='hfi_dir|thetaPark_enc').plot()


# df.filter(regex='emech').plot()

# df.filter(regex='T_FF').plot()

# df.filter(regex='state1.Iq').plot()
# df.filter(regex='state1.Id').plot()



# df['motor.state1.mechcontout'].plot()
# (df['motor.state1.T_FF_acc']+df['motor.state1.T_FF_vel']).plot()
# (df['motor.state2.T_FF_acc']+df['motor.state2.T_FF_vel']).plot()


# df.filter(regex='Iq_SP').plot()