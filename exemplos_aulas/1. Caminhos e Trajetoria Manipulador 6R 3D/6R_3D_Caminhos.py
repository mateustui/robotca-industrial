"""
 Autor: Prof. Lucas Vago Santana
 Curso: Robótica Industrial - Engenharia de Controle e Automação
 Instituição: Ifes - campus Linhares
 Revisado em: 22/05/2022
 
 # Modificação dos exemplos para controle direto da IDE Python
 # Utilize o PLAY e o CTRL+C do Spyder IDE para controlar a simulação sem necessidade de clicar nos botões do CoppeliaSim
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
_, J0Handle = vrep.simxGetObjectHandle(clientID,'J0',vrep.simx_opmode_oneshot_wait)
_, J1Handle = vrep.simxGetObjectHandle(clientID,'J1',vrep.simx_opmode_oneshot_wait)
_, J2Handle = vrep.simxGetObjectHandle(clientID,'J2',vrep.simx_opmode_oneshot_wait)
_, J3Handle = vrep.simxGetObjectHandle(clientID,'J3',vrep.simx_opmode_oneshot_wait)
_, J4Handle = vrep.simxGetObjectHandle(clientID,'J4',vrep.simx_opmode_oneshot_wait)
_, J5Handle = vrep.simxGetObjectHandle(clientID,'J5',vrep.simx_opmode_oneshot_wait)

# Frames
_, FrameEHandle = vrep.simxGetObjectHandle(clientID,'FrameE',vrep.simx_opmode_oneshot_wait)
_, FrameAlvoHandle = vrep.simxGetObjectHandle(clientID,'FrameAlvo',vrep.simx_opmode_oneshot_wait)
_, FrameAlvo2Handle = vrep.simxGetObjectHandle(clientID,'FrameAlvo2',vrep.simx_opmode_oneshot_wait)
_, FrameAlvo3Handle = vrep.simxGetObjectHandle(clientID,'FrameAlvo3',vrep.simx_opmode_oneshot_wait)

# Envia referência de ângulo para juntas
def Setar_Juntas_Robo(q0, q1, q2, q3, q4, q5): 
   #Aplica cálculos ao objeto no simulador
   vrep.simxSetJointTargetPosition(clientID, J0Handle, q0, vrep.simx_opmode_streaming   )
   vrep.simxSetJointTargetPosition(clientID, J1Handle, q1, vrep.simx_opmode_streaming   )
   vrep.simxSetJointTargetPosition(clientID, J2Handle, q2, vrep.simx_opmode_streaming   )
   vrep.simxSetJointTargetPosition(clientID, J3Handle, q3, vrep.simx_opmode_streaming   )
   vrep.simxSetJointTargetPosition(clientID, J4Handle, q4, vrep.simx_opmode_streaming   )
   vrep.simxSetJointTargetPosition(clientID, J5Handle, q5, vrep.simx_opmode_streaming   )

#Função que obtém a pose de um objeto da cena
#Simula algoritmos de localização global [Ex. Visão Computacional]
def Obter_Pose(handle): 
    _, pos = vrep.simxGetObjectPosition(clientID, handle,-1,vrep.simx_opmode_oneshot_wait)
    _, ori = vrep.simxGetObjectOrientation(clientID, handle,-1,vrep.simx_opmode_oneshot_wait)
  
    x = pos[0]   
    y = pos[1]
    z = pos[2]
    phi = ori[0]
    theta = ori[1]
    psi = ori[2]
    
    return x, y, z, phi, theta, psi

# Retorna matriz de rotação de objetos da simulação
def Obter_R(phi, theta, psi):
  # Fonte: https://www.coppeliarobotics.com/helpFiles/en/eulerAngles.htm
  R = np.array((
                  (                              np.cos(psi)*np.cos(theta),                             -np.cos(theta)*np.sin(psi),           np.sin(theta) ),
                  ( np.cos(phi)*np.sin(psi) + np.cos(psi)*np.sin(phi)*np.sin(theta), np.cos(phi)*np.cos(psi) - np.sin(phi)*np.sin(psi)*np.sin(theta), -np.cos(theta)*np.sin(phi) ),
                  ( np.sin(phi)*np.sin(psi) - np.cos(phi)*np.cos(psi)*np.sin(theta), np.cos(psi)*np.sin(phi) + np.cos(phi)*np.sin(psi)*np.sin(theta),  np.cos(phi)*np.cos(theta) )
              ))
  return R

def Obter_q(R):
  n = 0.5 * np.sqrt(R[0][0] + R[1][1] + R[2][2] + 1)
  E = 0.5 * np.array ((
                             ( np.sign(R[2][1] - R[1][2]) * np.sqrt(R[0][0] - R[1][1] - R[2][2] + 1) ),
                             ( np.sign(R[0][2] - R[2][0]) * np.sqrt(R[1][1] - R[2][2] - R[0][0] + 1) ),
                             ( np.sign(R[1][0] - R[0][1]) * np.sqrt(R[2][2] - R[0][0] - R[1][1] + 1) )
                          ))
  q = np.array(( (n), (E[0]), (E[1]), (E[2]) ))
  
  return q


def Obter_R_6_0(q0, q1, q2, q3, q4, q5):
  R = np.array((
                  ( (np.sin(q3)*np.cos(q4)*np.cos(q5) + np.sin(q5)*np.cos(q3))*np.sin(q0) - (np.sin(q3)*np.sin(q5)*np.sin(q1 + q2) + np.sin(q4)*np.cos(q5)*np.cos(q1 + q2) - np.sin(q1 + q2)*np.cos(q3)*np.cos(q4)*np.cos(q5))*np.cos(q0), -(np.sin(q3)*np.sin(q5)*np.cos(q4) - np.cos(q3)*np.cos(q5))*np.sin(q0) - (np.sin(q3)*np.sin(q1 + q2)*np.cos(q5) - np.sin(q4)*np.sin(q5)*np.cos(q1 + q2) + np.sin(q5)*np.sin(q1 + q2)*np.cos(q3)*np.cos(q4))*np.cos(q0), (np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + np.cos(q4)*np.cos(q1 + q2))*np.cos(q0) + np.sin(q0)*np.sin(q3)*np.sin(q4) ), 
                  ( -(np.sin(q3)*np.cos(q4)*np.cos(q5) + np.sin(q5)*np.cos(q3))*np.cos(q0) - (np.sin(q3)*np.sin(q5)*np.sin(q1 + q2) + np.sin(q4)*np.cos(q5)*np.cos(q1 + q2) - np.sin(q1 + q2)*np.cos(q3)*np.cos(q4)*np.cos(q5))*np.sin(q0), (np.sin(q3)*np.sin(q5)*np.cos(q4) - np.cos(q3)*np.cos(q5))*np.cos(q0) - (np.sin(q3)*np.sin(q1 + q2)*np.cos(q5) - np.sin(q4)*np.sin(q5)*np.cos(q1 + q2) + np.sin(q5)*np.sin(q1 + q2)*np.cos(q3)*np.cos(q4))*np.sin(q0), (np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + np.cos(q4)*np.cos(q1 + q2))*np.sin(q0) - np.sin(q3)*np.sin(q4)*np.cos(q0) ),
                  ( -np.sin(q3)*np.sin(q5)*np.cos(q1 + q2) + np.sin(q4)*np.sin(q1 + q2)*np.cos(q5) + np.cos(q3)*np.cos(q4)*np.cos(q5)*np.cos(q1 + q2), -np.sin(q3)*np.cos(q5)*np.cos(q1 + q2) - np.sin(q4)*np.sin(q5)*np.sin(q1 + q2) - np.sin(q5)*np.cos(q3)*np.cos(q4)*np.cos(q1 + q2), np.sin(q4)*np.cos(q3)*np.cos(q1 + q2) - np.sin(q1 + q2)*np.cos(q4) )
              ))
  return R

#Simula encoders absolutos nas juntas
def Obter_Angulo_Junta(handle): 
    _, q_medido = vrep.simxGetJointPosition(clientID,handle,vrep.simx_opmode_streaming  )
    return q_medido

######################################################################################
################################  TRAJETÓRIA #########################################
######################################################################################

# Dados da Trajetória 
Xi = np.zeros(3)
Xf = np.zeros(3)
Xd = np.zeros(3)
Xdp =  np.zeros(3)

Qi = np.zeros(4)
Qf = np.zeros(4)
Qd = np.zeros(4)

def Iniciar_Planejador(x, y, z, phi, theta, psi, xd, yd, zd, phid, thetad, psid):
     # Geração da Trajetória 
    Xi = np.array((
        (x), # Posições iniciais
        (y),
        (z)
    ))
     
    Xf = np.array((
        (xd), # Posições finais
        (yd),
        (zd)
    ))
    
    Ri = Obter_R(phi, theta, psi)  # Matriz de Rotação Inicial
    Qi = Obter_q(Ri) #Quaternio que representa Ri
    
    
    Rf = Obter_R(phid, thetad, psid) #Matriz de Rotação Final
    Qf = Obter_q(Rf) #Quaternio que representa Rf
    
    return Xi, Xf, Qi, Qf
    
def Planejador_Trajetoria(Xi, Xf, Qi, Qf, tt, T):
    #Cálculo de coeficientes do polinômio
    S = np.array((
                    (0), #valor inicial s(0) = 0
                    (1), #valor final   s(T) = 1
                    (0), #valor inicial sp
                    (0), #valor final sp
                    (0),
                    (0)
            ))
    
    M = np.array((
                    (      0,        0,        0,      0,   0,      1),
                    (   T**5,     T**4,     T**3,   T**2,   T,      1),
                    (      0,        0,        0,      0,   1,      0),
                    ( 5*T**4,   4*T**3,   3*T**2,    2*T,   1,      0),
                    (      0,        0,        0,      2,   0,      0),
                    (20*T**3,  12*T**2,      6*T,      2,   0,      0)
            ))       

    C = np.dot(np.linalg.inv(M),S)

    s =    C[0] * tt**5 +    C[1] * tt**4 +    C[2] * tt**3 +    C[3] * tt**2 +  C[4] * tt +  C[5]; 
    #sp = 5*C[0] * tt**4 +  4*C[1] * tt**3 +  3*C[2] * tt**2 +  2*C[3] * tt    +  C[4]; 

    Xd = (1-s)*Xi + s*Xf;  
    #Xdp =(Xf-Xi)*sp;
    
    #Qd = (1-s)*Qi + s*Qf;  
    alpha = np.arccos(Qi[0]*Qf[0]+Qi[1]*Qf[1]+Qi[2]*Qf[2]+Qi[3]*Qf[3])
    Qd = ( np.sin((1-s)*alpha)*Qi + np.sin(s*alpha)*Qf )/np.sin(alpha)
    
    Rd = quaternion_rotation_matrix(Qd)
    
    return Xd[0], Xd[1], Xd[2], Rd

def Obter_R_3_0(q0, q1, q2):
  R = np.array((
                  (np.sin(q1 + q2)*np.cos(q0), np.sin(q0), np.cos(q0)*np.cos(q1 + q2)),
                  (np.sin(q0)*np.sin(q1 + q2), -np.cos(q0), np.sin(q0)*np.cos(q1 + q2)),
                  (np.cos(q1 + q2), 0, -np.sin(q1 + q2))
              ))
  return R


def quaternion_rotation_matrix(Q):
    #Fonte: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def Cinematica_Inversa(xd, yd, zd, Rd):
    ######################################
    ## Cinemática Inversa de Orientação ##
    ######################################
    
    #Parâmetros do robô - Cinemática Inversa
    a0 = 0.3
    a1 = 0.5
    a2 = 0.5 + 0.05 #a2 + a3 (muda final do elo para centro do punho)
    a3 = 0.05 #considerado
    a4 = 0.05
    a5 = 0.02
   
    #Inicializa as variáveis localmente na função
    q0 = Obter_Angulo_Junta(J0Handle)
    q1 = Obter_Angulo_Junta(J1Handle)
    q2 = Obter_Angulo_Junta(J2Handle)
    q3 = Obter_Angulo_Junta(J3Handle)
    q4 = Obter_Angulo_Junta(J4Handle)
    q5 = Obter_Angulo_Junta(J5Handle)
    
    r = np.sqrt(xd**2+yd**2+zd**2)
    
    if r >= 0 and r <= a0+a1+a2+a3+a4+a5:
        
        ######################################
        ## Cinemática Inversa de Posição    ##
        ######################################
        d = (a4 + a5)
    
        R_6_0_d = Rd
        
        xp = xd - d * R_6_0_d[0,2] 
        yp = yd - d * R_6_0_d[1,2]  
        zp = zd - d * R_6_0_d[2,2]
        
        r1 = np.sqrt(xp**2+yp**2)
        r2 = zp - a0
        r3 = np.sqrt(r1**2+r2**2)
    
        G = np.arctan2(r2,r1)
        
        B = np.arccos((r3**2+a1**2-a2**2)/(2*r3*a1))
        
        q0 = np.arctan2(yp, xp)
        q1 = np.pi/2 - B - G
        q2 =  np.pi/2 - (np.arccos((a1**2+a2**2-r3**2)/(2*a1*a2)))
        
        ######################################
        ## Cinemática Inversa de Orientação ##
        ######################################
        R_3_0 = Obter_R_3_0(q0, q1, q2)
        
        R_6_3 = np.dot(np.linalg.inv(R_3_0), R_6_0_d)
        
        #print(np.array2string(R_6_3, formatter={'float_kind': '{0:.1f}'.format}))
        
        q4 = np.arccos(R_6_3[2,2])
        q3 = np.arctan2(R_6_3[1,2], R_6_3[0,2])
        q5 = np.pi - np.arctan2(R_6_3[2,1] , R_6_3[2,0])  
        
        isOK = 1
    else:
        isOK = 0
    
    return q0, q1, q2, q3, q4, q5, isOK


def main():
    estado=0
    flag=0
    #Controle do tempo da simulação e da Trajetória Cartesiana
    dt = 0.05
    t = 0
    tt = 0
    
    # Obtém Pose do Alvo
    xd, yd, zd, phid, thetad, psid = Obter_Pose(FrameAlvoHandle)
    xd2, yd2, zd2, phid2, thetad2, psid2 = Obter_Pose(FrameAlvo2Handle)
    xd3, yd3, zd3, phid3, thetad3, psid3 = Obter_Pose(FrameAlvo3Handle)
    
    # Obtém Pose do Efetuador Final
   
    x, y, z, phi, theta, psi = Obter_Pose(FrameEHandle)
    Xi, Xf, Qi, Qf = Iniciar_Planejador(x, y, z, phi, theta, psi, xd, yd, zd, phid, thetad, psid)
    
    
           
    tjt_isRunning = 1 # 1-Sim  0-Não (Supervisor no planejador de trajetória)
    
    while vrep.simxGetConnectionId(clientID) != -1:
        if estado ==0:
            #Loop de controle do robô
            t0 = time.perf_counter() #Controle de tempo
            
            T = 10 # Período de interpoção em segundos    
            
            if tjt_isRunning == 1:
                xd, yd, zd, Rd = Planejador_Trajetoria(Xi, Xf, Qi, Qf, tt, T)
                tt += dt
                
                #Aplica cinemática inversa
                q0_r, q1_r, q2_r, q3_r, q4_r, q5_r, isOK = Cinematica_Inversa(xd, yd, zd, Rd)
            
                # Seta referência das juntas
                Setar_Juntas_Robo(q0_r, q1_r, q2_r, q3_r, q4_r, q5_r)
                    
                if tt>= T: #Trajetória finalizada
                    tt = 0
                    tjt_isRunning =1
                    print("Fim") #Printa
                    estado=1
                    flag=0
                    
        elif estado==1:
            t0 = time.perf_counter()
            if flag==0:
                flag=1
                T2 = 3 # Período de interpoção em segundos 
                x, y, z, phi, theta, psi = Obter_Pose(FrameEHandle)
                Xi2, Xf2, Qi2, Qf2 = Iniciar_Planejador(x, y, z, phi, theta, psi, xd2, yd2, zd2, phid2, thetad2, psid2)
            if tjt_isRunning == 1:
                xd2, yd2, zd2, Rd2 = Planejador_Trajetoria(Xi2, Xf2, Qi2, Qf2, tt, T2)
                tt += dt
                
                #Aplica cinemática inversa
                q0_r, q1_r, q2_r, q3_r, q4_r, q5_r, isOK = Cinematica_Inversa(xd2, yd2, zd2, Rd2)
            
                # Seta referência das juntas
                Setar_Juntas_Robo(q0_r, q1_r, q2_r, q3_r, q4_r, q5_r)
                    
                if tt>= T2: #Trajetória finalizada
                    tt = 0
                    tjt_isRunning = 1
                    print("Fim2") #Printa
                    estado=2
                    flag=0
        elif estado==2:
            t0 = time.perf_counter() #Controle de tempo
            if flag==0:
                flag=1
                
                T3 = 5 # Período de interpoção em segundos 
                x, y, z, phi, theta, psi = Obter_Pose(FrameEHandle)
                Xi3, Xf3, Qi3, Qf3 = Iniciar_Planejador(x, y, z, phi, theta, psi, xd3, yd3, zd3, phid3, thetad3, psid3)
            if tjt_isRunning == 1:
                xd3, yd3, zd3, Rd3 = Planejador_Trajetoria(Xi3, Xf3, Qi3, Qf3, tt, T3)
                tt += dt
                
                #Aplica cinemática inversa
                q0_r, q1_r, q2_r, q3_r, q4_r, q5_r, isOK = Cinematica_Inversa(xd3, yd3, zd3, Rd3)
            
                # Seta referência das juntas
                Setar_Juntas_Robo(q0_r, q1_r, q2_r, q3_r, q4_r, q5_r)
                    
                if tt>= T3: #Trajetória finalizada
                    tt = 0
                    tjt_isRunning = 0
                    print("Fim3") #Printa
                    estado=0
                    flag=0

        
        
        
        
        
        
        
        

            
            
            
        #Disparo do trigger de simulação
        vrep.simxSynchronousTrigger(clientID); # Trigger next simulation step (Blocking function call)
        
        t += dt
        #Aguardando dt
        while(time.perf_counter()-t0 <= dt): _ # Loop de 50ms

try:
    main()
    
except KeyboardInterrupt:
    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    vrep.simxFinish(clientID)