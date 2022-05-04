import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)

######################################################################################
############################### FUNÇÕES DE ROTAÇÃO ###################################
######################################################################################

#Função que cria matrizes de rotação
def rot2d(ang,xt,yt):
  #Criação da matriz R 
  theta = np.radians(ang)
  ct = np.cos(theta)
  st = np.sin(theta)
  return np.array(( (ct, -st, xt), 
                    (st,  ct, yt),
                    (0 , 0  , 1 )))
  
######################################################################################
############################### APLICANDO AS ROTAÇÕES ################################
######################################################################################


Op = np.array((0,0,1)) #
Xp = np.array((6,0,1)) # 
Yp = np.array((0,3,1)) # 


R = rot2d(45,8,5) #Criação da matriz R 
R2 = rot2d(-30,2,-5) #Criação da matriz R 


print(R)
print(R2)





# Vetores dados no sistema {A}
Oa = np.array((0,0,1)) #Origem de {A}
Xa = np.array((10,0,1)) # seta do eixo x (vermelho)
Ya = np.array((0,10,1)) # seta do eixo y (azul)

Ob = np.dot(R,Oa) # Origem de {A} rotacionada para o sistema {B}
Xb = np.dot(R,Xa) # seta do eixo x (vermelho), rotacionada para o sistema {B}
Yb = np.dot(R,Ya) # seta do eixo y (azul), rotacionada para o sistema {B}


Oc = np.dot(R2,Oa) # Origem de {A} rotacionada para o sistema {B}
Xc = np.dot(R2,Xa) # seta do eixo x (vermelho), rotacionada para o sistema {B}
Yc = np.dot(R2,Ya) # seta do eixo y (azul), rotacionada para o sistema {B}

Va = np.array((6,3,1)) #seta do vetor (verde)



######################################################################################
############################### PLOT #################################################
######################################################################################
plt.figure(figsize=(10,10))
ax = plt.gca()

# Plota o vetor [v]
#ax.quiver(Oa[0], Oa[1], Va[0], Va[1], color='g', width = 0.01, angles='xy', scale_units='xy', scale=1)

ax.plot(Va[0],Va[1],'g*')


orig=np.array((0,0,1))


ax.quiver(Oa[0], Oa[1], Xa[0], Xa[1], color='r', width = 0.01, angles='xy', scale_units='xy', scale=1)
ax.quiver(Oa[0], Oa[1], Ya[0], Ya[1], color='b', width = 0.01, angles='xy', scale_units='xy', scale=1)

ax.quiver(Ob[0], Ob[1], Xb[0]-Ob[0], Xb[1]-Ob[1], color='r', width = 0.01, angles='xy', scale_units='xy', scale=1)
ax.quiver(Ob[0], Ob[1], Yb[0]-Ob[0], Yb[1]-Ob[1], color='b', width = 0.01, angles='xy', scale_units='xy', scale=1)


ax.quiver(Oc[0], Oc[1], Xc[0]-Oc[0], Xc[1]-Oc[1], color='r', width = 0.01, angles='xy', scale_units='xy', scale=1)
ax.quiver(Oc[0], Oc[1], Yc[0]-Oc[0], Yc[1]-Oc[1], color='b', width = 0.01, angles='xy', scale_units='xy', scale=1)

ax.xaxis.set_major_locator(MultipleLocator(1))
ax.yaxis.set_major_locator(MultipleLocator(1))


lim = 10
ax.set_xlim(-0.5, lim+1)
ax.set_ylim(-lim+3,lim)
ax.set_aspect('equal', 'box')

plt.grid(True)
plt.show()

print("Coordenadas do vetor no sistema {A}:")
print("[x]=[%f]\n[y]=[%f]" % (Va[0], Va[1]))

RVb = np.dot(np.linalg.inv(R),Va)
print("\nCoordenadas do vetor no sistema {B}:")
print("[x]=[%f]\n[y]=[%f]" % (RVb[0], RVb[1]))

RVc = np.dot(np.linalg.inv(R2) ,Va)
print("\nCoordenadas do vetor no sistema {C}:")
print("[x]=[%f]\n[y]=[%f]" % (RVc[0], RVc[1]))
print("\nVerifique no gráfico a coerência das coordenadas!")