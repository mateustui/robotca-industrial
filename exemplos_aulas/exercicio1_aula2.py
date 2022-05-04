# -*- coding: utf-8 -*-
"""
Created on Sun Apr  3 13:55:13 2022

@author: Mateus
"""

import matplotlib.pyplot as plt
import numpy as np


dt= 0.001
tf= 1
m=0.2
g=-9.8
y0=1

t= np.arange(0.0, tf, dt)

y=np.zeros(np.size(t))
yp=np.zeros(np.size(t))
ypp=np.linspace(g,g,np.size(t))
 
y[0]=y0

for k in range(0, (np.size(t))-1):
    yp[k+1] = yp[k] + ypp[k+1]*dt
    y[k+1]=y[k]+yp[k+1]*dt

plt.figure()
p0=plt.plot(t,y,label="Posição [$m$]")
plt.legend()
plt.show()

#plt.figure()
p0=plt.plot(t,yp,label="Velocidade [$m/s$]")
plt.legend()
plt.show()

#plt.figure()
p0=plt.plot(t,ypp,label="Aceleração [$m/s^2$]")
plt.legend()
plt.show()



for k in range(0, (np.size(t))-1):
    if(y[k]*y[k+1]<=0):
        print("O objeto atinge o solo no instante t=%.2f s com uma velocidade de %.2f m/s."%(t[k],yp[k]))