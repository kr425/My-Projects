# -*- coding: utf-8 -*-
"""
Created on Sat Feb 11 12:28:57 2017

@author: krlas_000
"""
import numpy as num
from numpy import *
import scipy.linalg as la
import numpy.matlib as M
import matplotlib.pyplot as plt
import control
# Make some matlib functions accessible directly at the top level via, e.g. rand(3,3)
from numpy.matlib import rand,zeros,ones,empty,eye
from scipy.integrate import odeint
from scipy.linalg import expm as sp_expm
from control import lyap
from numpy.linalg import matrix_rank


a1=8.79e-4 #cross section of pipe outlet m^2
a2=7.37e-4 
a3=6.35e-4 
a4=4.36e-4
A1=.06 #cross section of tanks m^2
A2=.06
A3=.06
A4=.06
k1=1 
k2=1
g=9.81 #gravity
s1=.3 #Flow parameters, rate of water flow
s2=.4
tf=5
dt=.01

#==============================================================================
# def Ju(u,t_f=tf,dt=dt,xEq=num.asarray([0.0,0.0,0.0,0.0,0.0,0.0]),x0=x0,uEq=num.asarray([0.0,0.0])):
#     uT = lambda t : u[int(t/dt),:]
#     P = 0.0*num.identity(6)
#     Q = 100.0*num.identity(6)
#     R = 1.0*num.identity(2)
#     t = 0
#     t_,x_ = lst.forward_euler(f=fLin,t=t_f,x=x0,ut=uT,dt=dt)
#     integral = num.dot(num.dot((x_[-1,:] - xEq).T,P),x_[-1,:] - xEq)
#     while t < t_f:
#         i = int(t/dt)
#         integral += (num.dot(num.dot((x_[i,:] - xEq).T,Q),x_[i,:] - xEq) + num.dot(num.dot((u[i,:]-uEq).T,R),(u[i,:]-uEq)))*dt
#         t += dt
#     return integral
#==============================================================================




def jacobian(g,X,n=None):
    if not n:
        n = len(X)
    h = num.finfo(float).eps**(1.0/4.0)
    e = num.identity(n)
    D = []
    for j in range(n):
        D.append((.5/h)*(g(X+h*e[j]) - g(X-h*e[j])))
    return num.array(D).T

def jacobianTrj(g,X,n=None,m=None):
    if not n:
        m_,n = X.shape
    if not m:
        m,n_ = X.shape
    h = num.finfo(float).eps**(1.0/4.0)
    e = num.identity(m)
    D = []
    for i in range(n):
        for j in range(m):
            #print (i,j)
            E = num.zeros((m,n))
            E[:,i] = e[j]
            D.append((.5/h)*(g(X+h*E) - g(X-h*E)))
    return num.array(D).T


def steepestDescent(J,x,DJ=None,alpha=.0001):
    if not DJ:
        
        DJ = jacobian
    diff = -1.0
    while diff < -1e-3:
        grad = DJ(J,x)
        x_ = x
        x = x - alpha*grad
        diff = J(x)-J(x_)
    return J(x),x,

def steepestDescentTrj(J,x,DJ=None,alpha=0.0232539043695,maxIter=30000):
    if not DJ:
        DJ = jacobianTrj
    diff = 1.0
    K = 0
    #Use norm of difference between costs as termination condition
    #There are other valid termination conditions, e.g. norm(DJ) > lambda
    #Also stop if we reach a certain number of max iterations,
    #because we don't want to wait all day.
    while la.norm(diff) > 1e-3 and K < maxIter:
        grad = DJ(J,x)
        m,n = x.shape
        #print(m)
        gradMat = num.zeros((m,n))
        #print(x.shape)
        for i in range(n):
            #print(x.shape)
            gradMat[:,i] = grad[i*m:(i+1)*m]
           # print(x.shape)
        x_ = x
        x = x - alpha*gradMat
        diff = J(x)-J(x_)
        K += 1
    return J(x),x

'''Linear Least Squares'''
def getH(B,t):
    H=np.zeros((len(t),5))
    for i in range(len(t)):
        H[i,:]=B(t[i])
    return H

def lls(z,t,B):
    H=getH(B,t)
    return np.dot(la.inv(np.dot(H.T,H)),np.dot(H.T,z))

'''Kalman Filter'''

def kalman_filter(x0,P0,A,B,u,F,Q,H,R,t_,z_):
  """
  input:
    x0 - n x 1 array; initial state mean
    P0 - n x n array; initial state covariance
    A - func : t -> n x n array; state transition
    B - func : t -> n x m array; control input matrix
    u - func : t -> m x 1 array; control input
    F - func : t -> n x k array; disturbance matrix
    Q - func : t -> k x k array; disturbance covariance
    H - func : t -> l x n array; measurement matrix
    R - func : t -> l x l array; measurement covariance
    t_ - N array; times
    z_ - l x N array; observations

  output:
    x_ - n x N array; estimated state
    P_ - n x n x N array; estimate covariance
    
  """
  P[:,:,0]=P0
  x_ = [x0]; P_ = [P0]
  for i in range (len(t_)-1): 
    x_n=num.dot(A,x0)+num.dot(B,u[i])
    P_n=num.dot(num.dot(A,P[:,:,i]),A.T)+num.dot(num.dot(F,Q),F.T)
    k=num.dot(num.dot(P_n,H.T),la.inv(num.dot(num.dot(H,P_n),H.T)+R))
    P_n=P_n-num.dot(num.dot(k,H),P_n)
    x_n=x_n+num.dot(k,(z_[0,i]-num.dot(H,x_n)))
    x0=x_n
    P[:,:,i+1]=P_n
    x_.append(x_n)
    P_.append(P_n)

  return num.hstack(x_),num.dstack(P_)
'''LQR Linear Quad Regulator'''
def tvCTLQR(A,B,Q,R,Pt,tf,dt=dt):
    t_ = 0.0
    K = []
    P = []
    while t_ < tf:
        P_ = la.solve_continuous_are(A(t_),B(t_),Q(t_),R(t_))
        K_ = num.dot(la.inv(R(t_)), num.dot(B(t_).T,P_))
        K.append(K_)
        P.append(P_)
        t_ += dt
    K_ = num.dot(la.inv(R(t_)), num.dot(B(t_).T,Pt))
    K.append(K_)
    P.append(Pt)
    return num.asarray(K),num.asarray(P)

def newtonRaphson(J,x,D=None):
    if not D:
        D = jacobian
    DJ = lambda x : D(J,x)
    D2J = lambda x : D(DJ,x)
    diff = -1.0
    while diff < 0:
        x_ = x      
        x = x - num.dot(la.inv(D2J(x)),DJ(x).T)
        diff = J(x)-J(x_)
        
    return J(x),x
    
def leastSquares(x):
    return la.norm(num.dot(A1,x) - b1)**2.0
    
def fwd_euler(f,t,x,u,dt=1e-2):
    j,t_l,x_l,u_l = 0,[0],[x],[u]
    while j*dt < t:
        t_l.append((j+1)*dt)
        x_l.append(x_l[-1] + dt*f(j*dt,x_l[-1]),u_l[u])#2x1+2x1
        j += 1
        u_l.append(t_l[-1],x_l[-1])
    return num.array(t_l),num.array(x_l),num.array(u_l)
    
    
def forward_euler(f,t,x,t0=0.,dt=dt,ut=None,ux=None,utx=None,return_u=False):
 
  t_,x_,u_ = [t0],[x],[]
  
  inumuts = sum([1 if u is not None else 0 for u in [ut,ux,utx]])
  assert inumuts <= 1, "more than one of ut,ux,utx defined"

  if inumuts == 0:
    assert not return_u, "no input supplied"
  else:
    if ut is not None:
      u = lambda t,x : ut(t)
    elif ux is not None:
      u = lambda t,x : ux(x)
    elif utx is not None:
      u = lambda t,x : utx(t,x)

  while t_[-1]+dt < t:
    if inumuts == 0:
      _t,_x = t_[-1],x_[-1]
      dx = f(t_[-1],x_[-1]) * dt
    else:
      _t,_x,_u = t_[-1],x_[-1],u(t_[-1],x_[-1])
      dx = f(_t,_x,_u) * dt
      u_.append( _u )

    x_.append( _x + dx )
    t_.append( _t + dt )

  if return_u:
    return num.asarray(t_),num.asarray(x_),num.asarray(u_)
  else:
    return num.asarray(t_),num.asarray(x_)

'''Four Tank System Equations'''
def f(t,x,u):
    h1,h2,h3,h4=x
    U1,U2=u 
    tank1=-a1/A1*num.sqrt(2*g*h1)+a3/A1*num.sqrt(2*g*h3)+(s1*k1*U1)/A1
    tank2=-a2/A2*num.sqrt(2*g*h2)+a4/A2*num.sqrt(2*g*h4)+(s2*k2*U2)/A2
    tank3=-a3/A3*num.sqrt(2*g*h3)+((1-s2)*k2*U2)/A3
    tank4=-a4/A4*num.sqrt(2*g*h4)+((1-s1)*k1*U1)/A4
    return num.array([tank1,tank2,tank3,tank4])
#Linearized form of the tank system
def linear(t,x,u):
    h1,h2,h3,h4=x[0],x[1],x[2],x[3]
    T1=(A1/a1)*num.sqrt(2*h1/g)    
    T2=(A2/a2)*num.sqrt(2*h2/g)  
    T3=(A3/a3)*num.sqrt(2*h3/g)  
    T4=(A4/a4)*num.sqrt(2*h4/g)  
    
    A=num.array([[-1/T1,0,A3/(A1*T3),0],
                   [0,-1/T2,0,A4/(A2*T4)],
                   [0,0,-1/T3,0],
                   [0,0,0,-1/T4]])
                   
    B=num.array([[s1/A1, 0],
                 [0,s1/A2],
                 [0,(1-s2)/A3],
                 [(1-s1)/A4,0]])
                 
    return num.dot(A,x)+num.dot(B,u)

def linear_noneq(t,xtrj,u):
    h1,h2,h3,h4=xtrj[int(t/dt)]
    T1=(A1/a1)*num.sqrt(2*h1/g)    
    T2=(A2/a2)*num.sqrt(2*h2/g)  
    T3=(A3/a3)*num.sqrt(2*h3/g)  
    T4=(A4/a4)*num.sqrt(2*h4/g)  
    
    A=num.array([[-1/T1,0,A3/(A1*T3),0],
                   [0,-1/T2,0,A4/(A2*T4)],
                   [0,0,-1/T3,0],
                   [0,0,0,-1/T4]])
                   
                 
    return A
B=num.array([[s1/A1, 0],
                 [0,s1/A2],
                 [0,(1-s2)/A3],
                 [(1-s1)/A4,0]])
    
def fLinNonEq(t,x,u):
    return num.dot(linear_noneq(t),x) + num.dot(B,u)
    



#############################################################################

t=10
b=100
q=1
r=100
q_=1
w=num.pi/10
v=2*w
''' Calculations'''
C2=0.5*t*u1**2 + 0.5*t*u2**2 + 0.79*u1**2*sin(0.628*t) - 2.12*u1*u2*sin(0.314*t)*sin(0.62*t)\
 + 4.24*u1*u2 - 1.06*u1*u2*sin(1.25*t)/sin(0.31*t) - 0.39*u2**2*sin(1.25*t)
J1a=q*(u1*t**2/2-b)**2+r*t*u1**2
J1b=q*(u1*t**2/2+u2*t**3/6-b)**2+r*t*(t**2*u2**2/3+t*u1*u2+u1**2)
J1c=q*(u1*t**2/2+u2*t**3/6-b)**2+q_*(u1*t+u2*t**2/2)**2+r*t*(t**2*u2**2/3+t*u1*u2+u1**2)
J1d=q*(-u1/v**2*num.cos(v*t)+u2/w**2*num.sin(w*t)-b)**2+r*C2
J1a_solve=solve([diff(J1a,u1)],[u1])
J1b_solve=solve([diff(J1b,u2),diff(J1b,u1)],[u2,u1])
J1c_solve=solve([diff(J1c,u2),diff(J1c,u1)],[u2,u1])
J1d_solve=solve([diff(J1d,u2),diff(J1d,u1)],[u2,u1])


'''Calculate Differentials from function'''
y=num.array([2,2])
fun=2*y1**2+3*y2*y1+3*y2**2
df1=diff(fun,y1)
df2=diff(fun,y2)
fun_1=sp.lambdify([y1, y2],fun)
df_1=sp.lambdify([y1,y2],df1)
df_2=sp.lambdify([y1,y2],df2)

'''g and f both compute the given function at the inputs'''
def g(y):
    y1,y2=y
    x=[]
    x.append(2*y1**2+3*y2*y1+3*y2**2)
    return num.array(x)
g3 = lambda x : x[0]**2 + x[1] + x[0]*x[1] + x[1]**2
def f(function,y):
    y1,y2=y
    x=[]
    x.append(function(y1,y2))
    return num.array(x)
def D(g,y,d=1e-4):
    ''' This function simulates the Jacobian of g 
    with respect to vector y using the finite central difference'''     
    I=num.identity(len(y))
    Dyg = []
    for j in range(len(y)):
      Dyg.append((.5/d)*(g3(y+d*I[j]) - g3(y-d*I[j])))
      #Dyg.append((.5/d)*(f(fun_1,y+d*I[j]) - f(fun_1,y-d*I[j])))
    return num.array(num.transpose(Dyg))
def D2(g,d1,d2,y,d=1e-4):
    ''' This function simulates the Jacobian of the first derivative (Hessian)
    with respect to vector y using the finite central difference'''     
    I=num.identity(len(y))
    Dyg = []
    for j in range(len(y)):
      Dyg.append((.5/d)*(g(d1,d2,y+d*I[j]) - g(d1,d2,y-d*I[j])))
    return num.array(num.transpose(Dyg))
def Dcomp2(df1,df2,y):
    ''' This function is the Actual derivative of "g"    '''
    y1,y2=y
    x=[]
    x2=[]
    x.append(df1(y1,y2))
    x2.append(df2(y1,y2))
    return num.array([x,x2]).T
def D_2(g,y,d):
    '''This is the second derivative just for the sake of it '''
    Dyg=[]
    I=num.identity(len(y))
    for j in range(len(y)):
       Dyg.append((1/d**2)*(g(y+d*I[j])-2*(g(y))+g(y-d*I[j])))
    return num.array(Dyg)
def Testsuite(g,R,C):
    '''This function tests the difference between the actual first
    derivative and the simulated derivative using finite central difference'''
    y=[]
    error=[]
    for j in range(10):
        y=num.array([j,j])
        J=R(df_1,df_2,y)
        J1=C(g,y,1e-4)
        error.append(J-J1)
    return error
test=Testsuite(g,Dcomp2,D)
'''The Following is both the test suite and comp of NR and steppest desc.'''
'''Based on the function the two algorithms will be close to zero when inputing there'''
'''values back into the Jacobian, but if the function is not quadriatic it will'''
'''give bad results'''


Hessian=D2(Dcomp2,df_1,df_2,y,1e-4)



