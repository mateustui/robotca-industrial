"""
 Autor: Prof. Lucas Vago Santana
 Curso: Robótica Industrial - Engenharia de Controle e Automação
 Instituição: Ifes - campus Linhares
 Revisado em: 22/05/2022
 
 # Modificação dos exemplos para controle direto da IDE Python
 # Utilize o PLAY e o CTRL+C do Spyder IDE para controlar a simulação sem 
   necessidade de clicar nos botões do CoppeliaSim
"""

import vrep
import time
import sys
import numpy as np

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
   print ('Servidor conectado!') 
else:
    print ('Problemas para conectar o servidor!')
    sys.exit()

#Ativa modo síncrono da RemoteAPI
vrep.simxSynchronous(clientID, True) 

#Inicia a simulação
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);


# Juntas do Robô
_, J1Handle = vrep.simxGetObjectHandle(clientID,'J1',vrep.simx_opmode_oneshot_wait)
_, J2Handle = vrep.simxGetObjectHandle(clientID,'J2',vrep.simx_opmode_oneshot_wait)

# Frame da Ferramenta
_, FrameEHandle = vrep.simxGetObjectHandle(clientID,'FrameE',vrep.simx_opmode_oneshot_wait)

# Alvo
_, AlvoHandle = vrep.simxGetObjectHandle(clientID,'Alvo',vrep.simx_opmode_oneshot_wait)

# Envia referência de ângulo para juntas
def Setar_Juntas_Robo(q1, q2): 
   #Aplica cálculos ao objeto no simulador
   vrep.simxSetJointTargetPosition(clientID, J1Handle, q1, vrep.simx_opmode_streaming)
   vrep.simxSetJointTargetPosition(clientID, J2Handle, q2, vrep.simx_opmode_streaming)

#Função que obtém a pose de um objeto da cena
#Simula algoritmos de localização global [Ex. Visão Computacional]
def Obter_Pose(handle): 
    _, pos = vrep.simxGetObjectPosition(clientID, handle,-1,vrep.simx_opmode_streaming)
    _, ori = vrep.simxGetObjectOrientation(clientID, handle,-1,vrep.simx_opmode_streaming)

    x = pos[0]   
    y = pos[1]
    z = pos[2]
    phi = ori[0]
    theta = ori[1]
    psi = ori[2]
    
    return x, y, z, phi, theta, psi

#Função que obtem o ângulo de uma junta
#Simula encoders absolutos nas juntas
def Obter_Angulo_Junta(handle): 
    _, q_medido = vrep.simxGetJointPosition(clientID,handle,vrep.simx_opmode_streaming)
    return q_medido

def main():
    #Inicialização
    dt = 0.05
    

    q1_c=0.0
    q2_c=0.0
    #Controle do tempo de simulação
    t = 0
    
    #Dimensões dos elos do robô
    a1 = 0.5
    a2 = 0.5
    
    #define o sinal do angulo
    controle=True
    flag = False

    #Loop de controle do robô
    while vrep.simxGetConnectionId(clientID) != -1:
        t0 = time.perf_counter() #Controle de tempo
        t+=dt
      
        # Obtém Pose do Alvo
        xd, yd, zd, phid, thetad, psid = Obter_Pose(AlvoHandle)
    
        x, y, z, phi, theta, psi = Obter_Pose(FrameEHandle)

        #Print de dados
        print("x: %1.2f  y: %1.2f  z: %1.2f" % (xd, yd, zd))
       # print("phi: %1.2f  theta: %1.2f  psi: %1.2f" % (phi*180/np.pi, theta*180/np.pi, psi*180/np.pi))
        #print("q1: %1.2f  q2: %1.2f" % (q1_m*180/np.pi,q2_m*180/np.pi))
        
        #Captura posição angular corrente
        q1_m = Obter_Angulo_Junta(J1Handle)
        q2_m = Obter_Angulo_Junta(J2Handle)
        

        if(xd!=0 and yd!=0):
            flag =True
        else:
            flag = False

        if flag:
            if controle:
                q2_c=np.arccos((xd*xd+yd*yd-a1*a1-a2*a2)/(2*a1*a2))
                #print(np.degrees(q2_c))
                q1_c=np.arctan(yd/xd)-np.arctan((a2*np.sin(q2_c))/(a1+a2*np.cos(q2_c)))
                #Setar_Juntas_Robo(q1_c, q2_c)
            else:
                q2_c=-np.arccos((xd*xd+yd*yd-a1*a1-a2*a2)/(2*a1*a2))
                q1_c=np.arctan(yd/xd)-np.arctan((a2*np.sin(q2_c))/(a1+a2*np.cos(q2_c)))
                #Setar_Juntas_Robo(q1_c, q2_c)
        
        #Programa aqui a cinemática inversa
        #q1_r = np.pi/8
        #q2_r = np.pi/4
        
        # Seta referência das juntas
        if xd<0:
            #q2_c=-q2_c
            q1_c=q1_c+np.pi


        Setar_Juntas_Robo(q1_c, q2_c)
        
        #Print de dados
        print("x: %1.2f  y: %1.2f  z: %1.2f" % (x, y, z))
        print("phi: %1.2f  theta: %1.2f  psi: %1.2f" % (phi*180/np.pi, theta*180/np.pi, psi*180/np.pi))
        print("q1: %1.2f  q2: %1.2f" % (q1_m*180/np.pi,q2_m*180/np.pi))
         
        #Disparo do trigger de simulação
        vrep.simxSynchronousTrigger(clientID); # Trigger next simulation step (Blocking function call)
        #Aguardando dt
        while(time.perf_counter()-t0 <= dt): _ # Loop de 50ms
    
try:
    main()
    
except KeyboardInterrupt:
    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    vrep.simxFinish(clientID)