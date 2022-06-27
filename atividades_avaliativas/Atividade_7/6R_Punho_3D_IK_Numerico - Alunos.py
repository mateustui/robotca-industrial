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

dt = 0.05

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
_, FrameEHandle = vrep.simxGetObjectHandle(clientID,'FrameE',vrep.simx_opmode_oneshot_wait)
# Alvo
_, AlvoHandle = vrep.simxGetObjectHandle(clientID,'Alvo',vrep.simx_opmode_oneshot_wait)

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

# Envia referência de ângulo para juntas
def Setar_Juntas_Robo(q0, q1, q2, q3, q4, q5): 
   #Aplica cálculos ao objeto no simulador
   vrep.simxSetJointTargetPosition(clientID, J0Handle, q0, vrep.simx_opmode_streaming)
   vrep.simxSetJointTargetPosition(clientID, J1Handle, q1, vrep.simx_opmode_streaming)
   vrep.simxSetJointTargetPosition(clientID, J2Handle, q2, vrep.simx_opmode_streaming)
   vrep.simxSetJointTargetPosition(clientID, J3Handle, q3, vrep.simx_opmode_streaming)
   vrep.simxSetJointTargetPosition(clientID, J4Handle, q4, vrep.simx_opmode_streaming)
   vrep.simxSetJointTargetPosition(clientID, J5Handle, q5, vrep.simx_opmode_streaming)

######################################################################################
############################  MATRIZES DE ROTAÇÃO ####################################
######################################################################################
def Obter_R_6_0(q0, q1, q2, q3, q4, q5):
  R = np.array((
                  ( (np.sin(q3)*np.cos(q4)*np.cos(q5) + np.sin(q5)*np.cos(q3))*np.sin(q0) - (np.sin(q3)*np.sin(q5)*np.sin(q1 + q2) + np.sin(q4)*np.cos(q5)*np.cos(q1 + q2) - np.sin(q1 + q2)*np.cos(q3)*np.cos(q4)*np.cos(q5))*np.cos(q0), -(np.sin(q3)*np.sin(q5)*np.cos(q4) - np.cos(q3)*np.cos(q5))*np.sin(q0) - (np.sin(q3)*np.sin(q1 + q2)*np.cos(q5) - np.sin(q4)*np.sin(q5)*np.cos(q1 + q2) + np.sin(q5)*np.sin(q1 + q2)*np.cos(q3)*np.cos(q4))*np.cos(q0), (np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + np.cos(q4)*np.cos(q1 + q2))*np.cos(q0) + np.sin(q0)*np.sin(q3)*np.sin(q4) ), 
                  ( -(np.sin(q3)*np.cos(q4)*np.cos(q5) + np.sin(q5)*np.cos(q3))*np.cos(q0) - (np.sin(q3)*np.sin(q5)*np.sin(q1 + q2) + np.sin(q4)*np.cos(q5)*np.cos(q1 + q2) - np.sin(q1 + q2)*np.cos(q3)*np.cos(q4)*np.cos(q5))*np.sin(q0), (np.sin(q3)*np.sin(q5)*np.cos(q4) - np.cos(q3)*np.cos(q5))*np.cos(q0) - (np.sin(q3)*np.sin(q1 + q2)*np.cos(q5) - np.sin(q4)*np.sin(q5)*np.cos(q1 + q2) + np.sin(q5)*np.sin(q1 + q2)*np.cos(q3)*np.cos(q4))*np.sin(q0), (np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + np.cos(q4)*np.cos(q1 + q2))*np.sin(q0) - np.sin(q3)*np.sin(q4)*np.cos(q0) ),
                  ( -np.sin(q3)*np.sin(q5)*np.cos(q1 + q2) + np.sin(q4)*np.sin(q1 + q2)*np.cos(q5) + np.cos(q3)*np.cos(q4)*np.cos(q5)*np.cos(q1 + q2), -np.sin(q3)*np.cos(q5)*np.cos(q1 + q2) - np.sin(q4)*np.sin(q5)*np.sin(q1 + q2) - np.sin(q5)*np.cos(q3)*np.cos(q4)*np.cos(q1 + q2), np.sin(q4)*np.cos(q3)*np.cos(q1 + q2) - np.sin(q1 + q2)*np.cos(q4) )
              ))
  return R

def Obter_R(phi, theta, psi):
  # https://www.coppeliarobotics.com/helpFiles/en/eulerAngles.htm
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

######################################################################################
############################ CINEMÁTICA INVERSA ######################################
######################################################################################
def cinematica_inv_num_Jt(xd, yd, zd, phid, thetad, psid, q0, q1, q2, q3, q4, q5):
    a0 = 0.3
    a1 = 0.5
    a2 = 0.5
    a3 = 0.05
    a4 = 0.05
    a5 = 0.02
    
    # Inicialização do vetor q
    q = np.array(( (q0),  
                   (q1),
                   (q2),
                   (q3),
                   (q4),
                   (q5)
                ))
    
    r = np.sqrt(xd**2+yd**2+zd**2)
    
    if r >= 0 and r <= a0+a1+a2+a3+a4+a5: # Workspace
        #Inicializa vetor Cinemática Direta de Posição
        Cd = np.array((
                        (  ),
                        (  ),
                        (  )
                    ))
                
        Ep = np.array (( ( xd - Cd[0] ), #Inicializa vetor erro de posição
                         ( yd - Cd[1] ),
                         ( zd - Cd[2] )
                      ))
        
        # Cálculo do erro de orientação de acordo com Sciavicco e Siciliano (2000, p. 111) - Eq. (3.87)
        # Matriz de rotação do alvo (Rd)
        R_d = Obter_R(phid, thetad, psid)
        
        # Quatérnio de Rd
        qd = Obter_q(R_d) 
        n_d = qd[0]
        E_d = 0.5 * np.array ((
                               ( qd[1] ),
                               ( qd[2] ),
                               ( qd[3] )
                            ))
        
        # Matriz de rotação do efetuador final - calculado a partir do vetor q (Rq)
        R_q = Obter_R_6_0(q0, q1, q2, q3, q4, q5)
        
        # Quatérnio de Rq
        qq = Obter_q(R_q) 
        n_q = qq[0]
        E_q = 0.5 * np.array ((
                               ( qq[1] ),
                               ( qq[2] ),
                               ( qq[3] )
                            ))
        
        # Operador antissimétrico com os elementos do vetor E_d
        S_E_d = np.array((
                (       0, -E_d[2],  E_d[1]),
                (  E_d[2],       0, -E_d[0] ),
                ( -E_d[1],  E_d[0],      0 )
                ))
        
        # Erro de orientação 
        Eo = np.dot(n_q,E_d) - np.dot(n_d,E_q) - np.dot(S_E_d, E_q)
        
        
        E = np.array (( ( Ep[0] ), #Inicializa vetor erro de posição e orientação espacial 3D
                        ( Ep[1] ),
                        ( Ep[2] ),
                        ( Eo[0] ), 
                        ( Eo[1] ),
                        ( Eo[2] )
                    ))
        
        # ReCalcula/Atualiza o Jacobiano em função de q
        J = np.array(( 
                       (  ),
                       (  ),
                       (  ),
                       (  ),
                       (  ),
                       (  )
                    ))

        
        # ReCalcula/Atualiza a inversa do Jacobiano numericamente
        Jt = np.transpose(J)
          
        K = np.array(( (10, 0, 0, 0, 0, 0), #matriz de ganhos
                       (0, 10, 0, 0, 0, 0), 
                       (0, 0, 10, 0, 0, 0),
                       (0, 0, 0, 10, 0, 0), 
                       (0, 0, 0, 0, 10, 0), 
                       (0, 0, 0, 0, 0, 10)
                    ))
          
        V =  np.dot(K, E) #termo auxiliar V = (Xdp + K * (Xd - X))
          
        qp = np.dot(Jt, V) # qp = J^T * (K * (Xd - X)) 
          
        #Atualiza q pela integração numérica q_k+1 = q_k + qp * dt 
        q = q +  qp * dt
        q = np.arctan2(np.sin(q), np.cos(q)) # Conversão para o domínio [-pi +pi]
        
        isOK = 1
    else:
        isOK = 0
    
    return q[0], q[1], q[2], q[3], q[4], q[5], isOK

def cinematica_inv_num_Ji(xd, yd, zd, phid, thetad, psid, q0, q1, q2, q3, q4, q5):
    a0 = 0.3
    a1 = 0.5
    a2 = 0.5
    a3 = 0.05
    a4 = 0.05
    a5 = 0.02
    # Inicialização do vetor q
    q = np.array(( (q0),  
                   (q1),
                   (q2),
                   (q3),
                   (q4),
                   (q5)
                ))
    
    r = np.sqrt(xd**2+yd**2+zd**2)
    
    if r >= 0 and r <= a0+a1+a2+a3+a4+a5: # Workspace
        #Inicializa vetor Cinemática Direta
        Cd = np.array ((
                         ( (a4 + a5)*np.sin(q0)*np.sin(q3)*np.sin(q4) + (a1*np.sin(q1) + a2*np.cos(q1 + q2) + a3*np.cos(q1 + q2) + a4*np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + a4*np.cos(q4)*np.cos(q1 + q2) + a5*np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + a5*np.cos(q4)*np.cos(q1 + q2))*np.cos(q0) ),
                         ( -(a4 + a5)*np.sin(q3)*np.sin(q4)*np.cos(q0) + (a1*np.sin(q1) + a2*np.cos(q1 + q2) + a3*np.cos(q1 + q2) + a4*np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + a4*np.cos(q4)*np.cos(q1 + q2) + a5*np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + a5*np.cos(q4)*np.cos(q1 + q2))*np.sin(q0) ), 
                         ( a0 + a1*np.cos(q1) - a2*np.sin(q1 + q2) - a3*np.sin(q1 + q2) + a4*np.sin(q4)*np.cos(q3)*np.cos(q1 + q2) - a4*np.sin(q1 + q2)*np.cos(q4) + a5*np.sin(q4)*np.cos(q3)*np.cos(q1 + q2) - a5*np.sin(q1 + q2)*np.cos(q4) )
                      ))
        
        Ep = np.array (( ( xd - Cd[0] ), #Inicializa vetor erro de posição
                         ( yd - Cd[1] ),
                         ( zd - Cd[2] )
                      ))
        
        # Cálculo do erro de orientação de acordo com Sciavicco e Siciliano (2000, p. 111) - Eq. (3.87)
        # Matriz de rotação do alvo (Rd)
        # Cálculo do erro de orientação de acordo com Sciavicco e Siciliano (2000, p. 111) - Eq. (3.87)
        # Matriz de rotação do alvo (Rd)
        R_d = Obter_R(phid, thetad, psid)
        
        # Quatérnio de Rd
        qd = Obter_q(R_d) 
        n_d = qd[0]
        E_d = 0.5 * np.array ((
                               ( qd[1] ),
                               ( qd[2] ),
                               ( qd[3] )
                            ))
        
        # Matriz de rotação do efetuador final - calculado a partir do vetor q (Rq)
        R_q = Obter_R_6_0(q0, q1, q2, q3, q4, q5)
        
        # Quatérnio de Rq
        qq = Obter_q(R_q) 
        n_q = qq[0]
        E_q = 0.5 * np.array ((
                               ( qq[1] ),
                               ( qq[2] ),
                               ( qq[3] )
                            ))
        
        # Operador antisimétrico com os elementos do vetor E_d
        S_E_d = np.array((
                (       0, -E_d[2],  E_d[1]),
                (  E_d[2],       0, -E_d[0] ),
                ( -E_d[1],  E_d[0],      0 )
                ))
        
        # Erro de orientação 
        Eo = np.dot(n_q,E_d) - np.dot(n_d,E_q) - np.dot(S_E_d, E_q)
        
        E = np.array (( ( Ep[0] ), #Inicializa vetor erro de posição
                        ( Ep[1] ),
                        ( Ep[2] ),
                        ( Eo[0] ), #Ignora erro de rotação
                        ( Eo[1] ),
                        ( Eo[2] )
                    ))
        
        # ReCalcula/Atualiza o Jacobiano em função de q
        J = np.array(( 
                       ( (a4 + a5)*np.sin(q3)*np.sin(q4)*np.cos(q0) - (a1*np.sin(q1) + a2*np.cos(q1 + q2) + a3*np.cos(q1 + q2) + a4*np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + a4*np.cos(q4)*np.cos(q1 + q2) + a5*np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + a5*np.cos(q4)*np.cos(q1 + q2))*np.sin(q0), (a1*np.cos(q1) - a2*np.sin(q1 + q2) - a3*np.sin(q1 + q2) + a4*np.sin(q4)*np.cos(q3)*np.cos(q1 + q2) - a4*np.sin(q1 + q2)*np.cos(q4) + a5*np.sin(q4)*np.cos(q3)*np.cos(q1 + q2) - a5*np.sin(q1 + q2)*np.cos(q4))*np.cos(q0), -(a2*np.sin(q1 + q2) + a3*np.sin(q1 + q2) - a4*np.sin(q4)*np.cos(q3)*np.cos(q1 + q2) + a4*np.sin(q1 + q2)*np.cos(q4) - a5*np.sin(q4)*np.cos(q3)*np.cos(q1 + q2) + a5*np.sin(q1 + q2)*np.cos(q4))*np.cos(q0), (a4*np.sin(q0)*np.cos(q3) - a4*np.sin(q3)*np.sin(q1 + q2)*np.cos(q0) + a5*np.sin(q0)*np.cos(q3) - a5*np.sin(q3)*np.sin(q1 + q2)*np.cos(q0))*np.sin(q4), a4*np.sin(q0)*np.sin(q3)*np.cos(q4) - a4*np.sin(q4)*np.cos(q0)*np.cos(q1 + q2) + a4*np.sin(q1 + q2)*np.cos(q0)*np.cos(q3)*np.cos(q4) + a5*np.sin(q0)*np.sin(q3)*np.cos(q4) - a5*np.sin(q4)*np.cos(q0)*np.cos(q1 + q2) + a5*np.sin(q1 + q2)*np.cos(q0)*np.cos(q3)*np.cos(q4), 0 ), 
                       ( (a4 + a5)*np.sin(q0)*np.sin(q3)*np.sin(q4) + (a1*np.sin(q1) + a2*np.cos(q1 + q2) + a3*np.cos(q1 + q2) + a4*np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + a4*np.cos(q4)*np.cos(q1 + q2) + a5*np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + a5*np.cos(q4)*np.cos(q1 + q2))*np.cos(q0), (a1*np.cos(q1) - a2*np.sin(q1 + q2) - a3*np.sin(q1 + q2) + a4*np.sin(q4)*np.cos(q3)*np.cos(q1 + q2) - a4*np.sin(q1 + q2)*np.cos(q4) + a5*np.sin(q4)*np.cos(q3)*np.cos(q1 + q2) - a5*np.sin(q1 + q2)*np.cos(q4))*np.sin(q0), -(a2*np.sin(q1 + q2) + a3*np.sin(q1 + q2) - a4*np.sin(q4)*np.cos(q3)*np.cos(q1 + q2) + a4*np.sin(q1 + q2)*np.cos(q4) - a5*np.sin(q4)*np.cos(q3)*np.cos(q1 + q2) + a5*np.sin(q1 + q2)*np.cos(q4))*np.sin(q0), -(a4*np.sin(q0)*np.sin(q3)*np.sin(q1 + q2) + a4*np.cos(q0)*np.cos(q3) + a5*np.sin(q0)*np.sin(q3)*np.sin(q1 + q2) + a5*np.cos(q0)*np.cos(q3))*np.sin(q4), -a4*np.sin(q0)*np.sin(q4)*np.cos(q1 + q2) + a4*np.sin(q0)*np.sin(q1 + q2)*np.cos(q3)*np.cos(q4) - a4*np.sin(q3)*np.cos(q0)*np.cos(q4) - a5*np.sin(q0)*np.sin(q4)*np.cos(q1 + q2) + a5*np.sin(q0)*np.sin(q1 + q2)*np.cos(q3)*np.cos(q4) - a5*np.sin(q3)*np.cos(q0)*np.cos(q4), 0 ), 
                       ( 0, -a1*np.sin(q1) - a2*np.cos(q1 + q2) - a3*np.cos(q1 + q2) - a4*np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) - a4*np.cos(q4)*np.cos(q1 + q2) - a5*np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) - a5*np.cos(q4)*np.cos(q1 + q2), -a2*np.cos(q1 + q2) - a3*np.cos(q1 + q2) - a4*np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) - a4*np.cos(q4)*np.cos(q1 + q2) - a5*np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) - a5*np.cos(q4)*np.cos(q1 + q2), -(a4 + a5)*np.sin(q3)*np.sin(q4)*np.cos(q1 + q2), a4*np.sin(q4)*np.sin(q1 + q2) + a4*np.cos(q3)*np.cos(q4)*np.cos(q1 + q2) + a5*np.sin(q4)*np.sin(q1 + q2) + a5*np.cos(q3)*np.cos(q4)*np.cos(q1 + q2), 0 ),
                       ( 0, -np.sin(q0), -np.sin(q0), np.cos(q0)*np.cos(q1 + q2), np.sin(q0)*np.cos(q3) - np.sin(q3)*np.sin(q1 + q2)*np.cos(q0), (np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + np.cos(q4)*np.cos(q1 + q2))*np.cos(q0) + np.sin(q0)*np.sin(q3)*np.sin(q4) ), 
                       ( 0, np.cos(q0), np.cos(q0), np.sin(q0)*np.cos(q1 + q2), -np.sin(q0)*np.sin(q3)*np.sin(q1 + q2) - np.cos(q0)*np.cos(q3), (np.sin(q4)*np.sin(q1 + q2)*np.cos(q3) + np.cos(q4)*np.cos(q1 + q2))*np.sin(q0) - np.sin(q3)*np.sin(q4)*np.cos(q0) ), 
                       ( 1, 0, 0, -np.sin(q1 + q2), -np.sin(q3)*np.cos(q1 + q2), np.sin(q4)*np.cos(q3)*np.cos(q1 + q2) - np.sin(q1 + q2)*np.cos(q4) )              
                    ))
        
        # ReCalcula/Atualiza a inversa do Jacobiano numericamente
        Ji = np.linalg.pinv(J)
          
        K = np.array(( (20, 0, 0, 0, 0, 0), #matriz de ganhos
                       (0, 20, 0, 0, 0, 0), 
                       (0, 0, 20, 0, 0, 0),
                       (0, 0, 0, 20, 0, 0), 
                       (0, 0, 0, 0, 20, 0), 
                       (0, 0, 0, 0, 0, 20)
                    ))
        
        # Aplica lei de controle qp = J(q)^-1 * (Xdp + K * (Xd - X)) 
        xdp = 0 # xd  constante, logo xdp = 0
        ydp = 0 # yd  constante, logo ydp = 0
        zdp = 0
        wxp = 0 
        wyp = 0 
        wzp = 0
        
        Xdp = np.array(( (xdp), #vetor de derivadas dos estados desejados
                         (ydp),
                         (zdp),
                         (wxp), 
                         (wyp),
                         (wzp)
                      ))
        
        V =  Xdp + np.dot(K, E) #termo auxiliar V = (Xdp + K * (Xd - X))
          
        qp = np.dot(Ji, V) # qp = J(q)^-1 * (Xdp + K * (Xd - X)) 
          
        #Atualiza q pela integração numérica q_k+1 = q_k + qp * dt 
        q = q +  qp * dt
        q = np.arctan2(np.sin(q), np.cos(q)) # Conversão para o domínio [-pi +pi]
        
        isOK = 1
    else:
        isOK = 0
    
    return q[0], q[1], q[2], q[3], q[4], q[5], isOK


def main():
    #Controle do tempo de simulação
    t = 0
     
    while vrep.simxGetConnectionId(clientID) != -1:
        t0 = time.perf_counter() #Controle de tempo
        t+=dt    
        # Obtém Pose do Alvo
        xd, yd, zd, phid, thetad, psid = Obter_Pose(AlvoHandle)
    
        #Obtém pose da ferramenta
        x, y, z, phi, theta, psi = Obter_Pose(FrameEHandle)
        
       
        #Captura posição angular corrente
        q0_m = Obter_Angulo_Junta(J0Handle)
        q1_m = Obter_Angulo_Junta(J1Handle)
        q2_m = Obter_Angulo_Junta(J2Handle)
        q3_m = Obter_Angulo_Junta(J3Handle)
        q4_m = Obter_Angulo_Junta(J4Handle)
        q5_m = Obter_Angulo_Junta(J5Handle)
        
        #Aplica cinemática inversa
        q0_r, q1_r, q2_r, q3_r, q4_r, q5_r, isOK = cinematica_inv_num_Ji(xd, yd, zd, phid, thetad, psid, q0_m, q1_m, q2_m, q3_m, q4_m, q5_m)
        
        # Seta referência das juntas
        Setar_Juntas_Robo(q0_r, q1_r, q2_r, q3_r, q4_r, q5_r)
    
        #Print de dados
        #print("xd: %1.2f  yd: %1.2f  zd: %1.2f" % (xd, yd, zd))
        #print(" x: %1.2f   y: %1.2f   z: %1.2f" % (x, y, z))
        
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
    
    

    





