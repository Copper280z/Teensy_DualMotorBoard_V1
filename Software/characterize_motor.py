#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thur Feb 22 15:25:45 2024

@author: bob
"""

import matplotlib.pyplot as plt
import pandas as pd
import struct
import numpy as np
import time
import control as ct
import TeensyMotorControl as tc

pi = np.pi
m = tc.Motor()
motor = tc.MotorVariables(m)

z = ct.TransferFunction( [1, 0] , [1] , float(m.Ts))

# set some alpha voltage and measure the current
def est_phase_resistance(m, v_min=0.01, v_max=1, n_steps=3):
    signals = ['motor.state1.Ialpha']
    m.setTrace(signals)
    test_voltages = np.linspace(v_min, v_max, n_steps)
    measured_resistance = []
    for volts in test_voltages:
        m.setpar('motor.state1.Valpha_offset', volts)
        amps = m.trace(0.3, outtype='arr').astype(np.float64)
        m.setpar('motor.state1.Valpha_offset', 0)
        mean_amps = np.mean(amps)
        measured_resistance.append(volts/mean_amps)
    return np.mean(measured_resistance), mean_amps

def measure_electrical_plant_open_loop(m, volts=1, Valpha_offset=0, Id_offset=0):
    m.setpar('s1.hfi_method', 0)
    m.setpar('motor.conf1.anglechoice', 0)
    
    NdownsamplePRBS = 3
    N = 30*NdownsamplePRBS*2047
    signals = ['motor.state1.Id_meas', 'motor.state1.Iq_meas',
               'motor.state1.dist', 'motor.state1.Vq', 'motor.state1.Vd']
    m.setTrace(signals )

    m.setpar('motor.state1.Id_offset_SP',Id_offset)
    
    m.setpar('motor.state1.Valpha_offset', Valpha_offset)
    if Valpha_offset != 0:
        time.sleep(0.5)
    m.setpar('motor.state1.Vq_distgain', 1)
    m.setpar('motor.state1.Vd_distgain', 1)
    m.setpar('motor.state1.Iq_distgain', 0)
    m.setpar('motor.state1.Id_distgain', 0)
    m.setpar('motor.state1.mechdistgain', 0)
    
    m.setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
    m.setpar('motor.state1.distval', volts)  # disturbance amplitude
    m.setpar('motor.state1.distoff', 0)  # disturbance offset
    df = m.trace(N * m.Ts)
    m.setpar('motor.state1.Valpha_offset', 0)
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
    
    plt.figure()
    m.bode( Pd , f, 'Measured D axis plant')
    m.bode( Pq , f, 'Measured Q axis plant')
     
    plt.figure(2)
    m.bode( 1 / Sd - 1 , f, 'Open loop D')
    m.bode( 1 / Sq - 1 , f, 'Open loop Q')
    
    Ld_arr = np.abs(1/(Pd * f * 2 * np.pi)) * 1e6
    Lq_arr = np.abs(1/(Pq * f * 2 * np.pi)) * 1e6
    Ld = np.mean(Ld_arr[f>4000])
    Lq = np.mean(Lq_arr[f>4000])
    
    plt.figure()
    plt.plot(
        f, Ld_arr, label=f'Ld [uH]\nmean: {np.mean(Ld_arr[f>2000]):.3f}')
    plt.plot(
        f, Lq_arr, label=f'Lq [uH]\nmean: {np.mean(Lq_arr[f>2000]):.3f}')
    plt.grid()
    plt.xlim([1e3, 10e3])
    plt.ylim([ np.min([Ld_arr[f>1000],Lq_arr[f>1000]])/1.5 , np.max([Ld_arr[f>1000],Lq_arr[f>1000]])*1.5])
    plt.title(f'DQ Inductance')
    plt.legend()
    
    plt.figure()
    plt.plot(
        f, (np.abs(1/(Pd * f * 2 * np.pi))) / np.abs(1/(Pq * f * 2 * np.pi)) )
    plt.grid()
    plt.xlim([1e3, 10e3])
    plt.ylim([ 0.5 , 1.5])
    plt.title('Ld/Lq ')
    
    return Ld/1e6, Lq/1e6

est_params = {}
R, max_amps = est_phase_resistance(m, v_min=0.1, v_max=1, n_steps=4)
est_params['R'] = R
print(f'Measured Resistance: {R:.4f}')
print(f'Observed current: {max_amps:.3f}')

m.setpar('motor.state1.R', est_params['R'])
Ld, Lq = measure_electrical_plant_open_loop(m, volts=R, Valpha_offset=0.0, Id_offset=0)
print(f'Measured Ld: {Ld*1e6:.3f} uH')
print(f'Measured Lq: {Lq*1e6:.3f} uH')
est_params['Ld'] = Ld
est_params['Lq'] = Lq

m.setpar('motor.conf1.Lq', Lq)
m.setpar('motor.conf1.Ld', Ld)
m.setpar('motor.state1.R', R)
