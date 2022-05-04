import matplotlib.pyplot as plt
import numpy as np

dt= 0.1
tf= 35

t= np.arange(0.0, tf, dt)

x=np.zeros(np.size(t))
xp=np.zeros(np.size(t))
xpp=np.zeros(np.size(t))

u=100
m=1000
b=50

for k in range(0, (np.size(t))-1):
    xpp[k+1]= u/m - b/m*xp[k]
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

for k in range(0, (np.size(t))-1):
    if(t[k]==12.0):
        print("A posicação do veículo no t=12s ser de:%.2f m"%x[k])

for k in range(0, (np.size(t))-1):
    if(t[k]==22.0):
        print("A velocidade do veículo no t=22s ser de:%.2f m/s"%xp[k])
        
for k in range(0, (np.size(t))-1):
    if(t[k]==32.0):
        print("A força de arrasto do veículo no t=12s ser de:%.2f m/s"%(xp[k]*b))
        