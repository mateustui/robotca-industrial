# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 15:09:16 2022

@author: Mateus
"""

import matplotlib.pyplot as plt
import numpy as np


dt= 0.1
c = 0.25
m = 1
const = 0.1
tf = 40.1
t= np.arange(0.0, tf, dt)

x=np.zeros(np.size(t))
xp=np.zeros(np.size(t))
xpp=np.zeros(np.size(t))

x[0]=0.5

for k in range(0, (np.size(t))-1):
    xpp[k+1]=-(const*x[k])-(c*xp[k])
    xp[k+1] = xp[k] + xpp[k+1]*dt
    x[k+1]=x[k]+xp[k+1]*dt

xpp[0]=xpp[1]
plt.figure()
p0=plt.plot(t,x,label="Posição [$m$]")
plt.legend()
plt.show()

#plt.figure()
p0=plt.plot(t,xp,label="Velocidade [$m/s$]")
plt.legend()
plt.show()

#plt.figure()
p0=plt.plot(t,xpp,label="Aceleração [$m/s^2$]")
plt.legend()
plt.show()

for l in range(0, (np.size(t))-1):
    if t[l]==10.0:
        print(x[l])
