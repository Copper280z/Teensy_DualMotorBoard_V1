#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 23 10:14:19 2024

@author: bob
"""
#%%
m.setpar('s1.hfi_method', 0)
m.setpar('motor.conf1.anglechoice', 0)

NdownsamplePRBS = 3
N = 30*NdownsamplePRBS*2047
signals = ['motor.state1.Id_meas', 'motor.state1.Iq_meas',
           'motor.state1.dist', 'motor.state1.Vq', 'motor.state1.Vd']
m.setTrace(signals )


gain = 5

Id = np.arange(-0.3,0.3,0.05)
Iq = np.arange(-0.3,0.3,0.05)
results = []
for Id_test in Id:
    time.sleep(0.5)
    for Iq_test in Iq:
        print(f'Id: {Id_test:.2f} - Iq: {Iq_test:.2f}')
        m.setpar('motor.state1.Id_offset_SP',Id_test)
        m.setpar('motor.state1.Iq_offset_SP',Iq_test)
        
        m.setpar('motor.state1.Vq_distgain', 1)
        m.setpar('motor.state1.Vd_distgain', 1)
        m.setpar('motor.state1.Iq_distgain', 0)
        m.setpar('motor.state1.Id_distgain', 0)
        m.setpar('motor.state1.mechdistgain', 0)
        
        m.setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
        m.setpar('motor.state1.distval', gain)  # disturbance amplitude
        m.setpar('motor.state1.distoff', 0)  # disturbance offset
        df = m.trace(N * m.Ts)
        m.setpar('motor.state1.distval', 0)  # disturbance amplitude
        m.setpar('motor.state1.distoff', 0)  # disturbance offset
        m.setpar('motor.conf.NdownsamplePRBS', 1)  # Downsampling
        
        m.setpar('motor.state1.Id_offset_SP',0.0)
        m.setpar('motor.state1.Iq_offset_SP',0.0)
        results.append({'test_current':[Id_test,Iq_test], 'data':df})
        time.sleep(0.1)

inductance=[]
for i,ret in enumerate(results):
    dfout = m.getFFTdf(ret['data'], NdownsamplePRBS , 10*2047 )
    f = dfout.index.values
    Pdd = dfout['motor.state1.Id_meas'].values / dfout['motor.state1.Vd'].values
    Pdq = dfout['motor.state1.Id_meas'].values / dfout['motor.state1.Vq'].values
    Pqq = dfout['motor.state1.Iq_meas'].values / dfout['motor.state1.Vq'].values
    Pqd = dfout['motor.state1.Iq_meas'].values / dfout['motor.state1.Vd'].values
    Sd = dfout['motor.state1.Vd'].values
    Sq = dfout['motor.state1.Vq'].values
    results[i]['ft_data'] = dfout
    
    Ldd_arr = np.abs(1/(Pdd * f * 2 * np.pi)) * 1e6
    Ldq_arr = np.abs(1/(Pdq * f * 2 * np.pi)) * 1e6
    Lqq_arr = np.abs(1/(Pqq * f * 2 * np.pi)) * 1e6
    Lqd_arr = np.abs(1/(Pqd * f * 2 * np.pi)) * 1e6
    inductance.append([ret['test_current'][0],
                       ret['test_current'][1], 
                       np.mean(Ldd_arr[200:]), 
                       np.mean(Lqq_arr[200:]), 
                       np.mean(Ldq_arr[200:]),
                       np.mean(Lqd_arr[200:])])

inductance = np.asarray(inductance)
Ldiff = inductance[:,3] - inductance[:,2]
Ldiff = Ldiff.reshape((len(Id), len(Iq)))

extent = [min(Iq),max(Iq),min(Id),max(Id)]
plt.figure()
plt.imshow(Ldiff,origin='lower', extent=extent)
plt.xlabel('Iq (amps)')
plt.ylabel('Id (amps)')
plt.title('Lq-Ld')
plt.grid(None)
plt.colorbar()
plt.show()

plt.figure()
plt.imshow(inductance[:,2].reshape((len(Id), len(Iq))),origin='lower', extent=extent)
plt.xlabel('Iq (amps)')
plt.ylabel('Id (amps)')
plt.title('Ldd')
plt.grid(None)
plt.colorbar()
plt.show()

plt.figure()
plt.imshow(inductance[:,3].reshape((len(Id), len(Iq))),origin='lower', extent=extent)
plt.xlabel('Iq (amps)')
plt.ylabel('Id (amps)')
plt.title('Lqq')
plt.grid(None)
plt.colorbar()
plt.show()

plt.figure()
plt.imshow(inductance[:,4].reshape((len(Id), len(Iq))),origin='lower', extent=extent)
plt.xlabel('Iq (amps)')
plt.ylabel('Id (amps)')
plt.title('Ldq')
plt.grid(None)
plt.colorbar()
plt.show()

plt.figure()
plt.imshow(inductance[:,5].reshape((len(Id), len(Iq))),origin='lower', extent=extent)
plt.xlabel('Iq (amps)')
plt.ylabel('Id (amps)')
plt.title('Lqd')
plt.grid(None)
plt.colorbar()
plt.show()

plt.figure()
plt.imshow(inductance[:,0].reshape((len(Id), len(Iq))),origin='lower', extent=extent)
plt.xlabel('Iq (amps)')
plt.ylabel('Id (amps)')
plt.title('Id')
plt.grid(None)
plt.show()

ax = plt.figure().add_subplot(projection='3d')
ax.plot_trisurf(inductance[:,0], inductance[:,1], Ldiff.ravel(),cmap='viridis', antialiased=True)
plt.show()