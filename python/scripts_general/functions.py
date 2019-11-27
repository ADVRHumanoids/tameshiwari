# -*- coding: utf-8 -*-

#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia
#
#   The script contains a number of useful functions and classes that are 
#   through out the Tameshiwari repo. The script will be populated in a 
#   continous way throughout the research period.
#   

from casadi import collocation_points
from casadi import DM
import numpy as np
import numpy.matlib as ml
import matplotlib.pyplot as plt
import matplotlib2tikz
import scipy.io as sio
from datetime import datetime
import os, sys
import __main__ as main
from cycler import cycler

#   COLORMAP
prop_cycle = plt.rcParams['axes.prop_cycle']
colors = prop_cycle.by_key()['color']
colors.extend(colors)
plt.rc('axes',prop_cycle=(cycler(color=colors)))

class ColMatrices:
    def __init__(self,d):
        self.d = d
        self.B = np.zeros(self.d+1)
        self.C = np.zeros([self.d+1,self.d+1]) 
        self.D = np.zeros(self.d+1)
        self.tau = np.append(0, collocation_points(self.d, 'legendre'))
        
        # Construct polynomial basis
        for j in range(self.d+1):
            # Construct Lagrange polynomials to get the polynomial basis at the collocation point
            p = np.poly1d([1])
            for r in range(self.d+1):
                if r != j:
                    p *= np.poly1d([1, -self.tau[r]]) / (self.tau[j]-self.tau[r])
        
            # Evaluate the polynomial at the final time to get the coefficients of the continuity equation
            self.D[j] = p(1.0)
        
            # Evaluate the time derivative of the polynomial at all collocation points to get the coefficients of the continuity equation
            pder = np.polyder(p)
            for r in range(self.d+1):
                self.C[j,r] = pder(self.tau[r])
        
            # Evaluate the integral of the polynomial to get the coefficients of the quadrature function
            pint = np.polyint(p)
            self.B[j] = pint(1.0)

class SolverParam:
    def __init__(self,stats={},N=0,h=0,T=[],Tf=0,fval=0):
        self.dict = {
            'N': N, 'h': h,
            'T': T.flatten(), 'Tf': Tf, 
            'fval': fval,
            'nIter': stats['iter_count'],
            'retStat': stats['return_status'],
            'stats': stats
        }

    def saveMat(self,dirName='',fileName='',suffix='SolverParam_'):
        if not dirName:
            dirName = os.getcwd() + '/results'
        if not os.path.isdir(dirName):
            os.mkdir(dirName)
        if not fileName:
            fileName = os.path.basename(main.__file__)
        if ".py" in fileName:
            fileName, file_ext = os.path.splitext(fileName)
        str_time = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        fileName = "%s/%s%s_%s.mat" % (dirName,suffix,fileName,str_time)
        sio.savemat(fileName,self.dict)

class RobotPose:
    def __init__(self,name='',q=np.array([]),qdot=np.array([]),tau=np.array([]),rate=10,qddot=np.array([])):
        # Make sure q, qdot, tau are matrices of N x nj
        # N is number of samples and nj is number of joints
        # self has: name, q, qdot, qddot, tau, rate, nj,
        #       N, Tf, T
        self.name = name
        self.q = q
        self.qdot = qdot
        self.qddot = qddot
        self.tau = tau
        self.rate = float(rate)
        self.nj = 1         # default number of joints = 1

        # Determine the amount of time samples and final time
        if np.shape(self.q)[0] > 1:
            self.N = np.shape(self.q)[0]
            self.Tf = (float(self.N-1))/rate
            self.T = np.matlib.linspace(0,self.Tf,self.N)

        # Determine the number of joints dependent on q
        if np.ndim(self.q) > 1 and np.shape(self.q)[1] > 1:      
            self.nj = np.shape(self.q)[1]
            if not self.name:
                self.name = []
                for j in range(self.nj):
                    tmp = "J%02d" %(j+1)
                    self.name.append(tmp)
                
    def interpolate(self,drate,method=''):
        # qdot needs to be provided

        # This function interpolates the known trajectory and adds more intermediate
        # points to it to have smooth motion in rviz
        # Techniques used are from Siciliano book chapter 4 Trajectory Planning

        # First we compare the rate of the instance with the desired rate.
        # If the desired rate is smaller or equal than self.rate then nothing will be processed
        if self.rate < drate and np.shape(self.qdot)[0] == np.shape(self.q)[0]:
            # The size of q, qdot and tau will be greater than before.
            # tau will be interpolated using zero order-hold technique
            N_new = int(self.Tf*drate + 1)
            T_new = np.matlib.linspace(0,self.Tf,N_new)
            # print "T vector of original: %s" %self.T
            # print "T vector of new: %s" %T_new
            q_new = np.zeros([N_new,self.nj])
            qdot_new = np.zeros([N_new,self.nj])

            # Algorithm: interpolating polynomials with imposed velocities at path points
            for j in range(self.nj):
                for k in range(self.N-1):
                    qi = self.q[k,j]
                    qf = self.q[k+1,j]
                    qdoti = self.qdot[k,j]
                    qdotf = self.qdot[k+1,j]
                    ti = self.T[k]
                    tf = self.T[k+1]

                    Ar0 = [1, ti, ti**2, ti**3]
                    Ar1 = [0, 1, 2*ti, 3*ti**2]
                    Ar2 = [1, tf, tf**2, tf**3]
                    Ar3 = [0, 1, 2*tf, 3*tf**2]
                    A = np.array([Ar0,Ar1,Ar2,Ar3],dtype = 'float')
                    b = np.array([qi,qdoti,qf,qdotf])
                    x = np.linalg.solve(A,b)
                    xdot = np.polyder(x)
                    x = np.flipud(x)
                    xdot = np.flipud(xdot)
                    
                    ind = np.nonzero((T_new >= self.T[k]) & (T_new < self.T[k+1]))[0]
                    sz = np.size(ind)
                    # TODO: add qddot & tau as well
                    for i in range(sz):
                        q_new[ind[i],j] = np.polyval(x,T_new[ind[i]])
                        qdot_new[ind[i],j] = np.polyval(xdot,T_new[ind[i]])
                    if k == self.N-2:
                        q_new[-1,j] = np.polyval(x,T_new[-1])
                        qdot_new[-1,j] = np.polyval(xdot,T_new[-1])
            self.q = q_new
            self.qdot = qdot_new
            self.tau = np.zeros([N_new,self.nj])
            self.T = T_new
            self.rate = drate
            self.N = N_new
            print "The frame rate has been increased."
        elif self.rate > drate:
            # This function doesn't require qdot, so no condition is set on this
            
            # print "number of frames by RobotPose: %s" % self.N
            # print "Final time by RobotPose: %s" %self.Tf
            N_new = int(self.Tf*drate + 1)
            T_new = np.matlib.linspace(0,self.Tf,N_new)
            q_new = np.zeros([N_new,self.nj])
            qdot_new = np.zeros([N_new,self.nj])
            for j in range(self.nj):
                for k in range(self.N-1):
                    ind = np.nonzero((T_new >= self.T[k]) & (T_new < self.T[k+1]))[0]
                    if len(ind) > 0:
                        q_new[ind,j] = self.q[k,j]
                        qdot_new[ind,j] = self.qdot[k,j]
                    if k == self.N-2:
                        q_new[-1,j] = self.q[-1,j]
                        qdot_new[-1,j] = self.qdot[-1,j]
            self.q = q_new
            # print len(self.q)
            self.qdot = qdot_new
            self.T = T_new
            self.rate = drate
            self.N = N_new
            print "The frame rate has been decreased."
        else:
            print "The frame rate is already the same as rviz"

    def saveMat(self,dirName='',fileName='',suffix='RobotPose_'):
        if not dirName:
            dirName = os.getcwd() + '/results'
        if not os.path.isdir(dirName):
            os.mkdir(dirName)
        if not fileName:
            fileName = os.path.basename(main.__file__)
        if ".py" in fileName:
            fileName, file_ext = os.path.splitext(fileName)
        saveDict = {
            'name':self.name,
            'q':self.q,
            'qdot':self.qdot,
            'qddot':self.qddot,
            'tau':self.tau,
            'rate':self.rate
        }
        str_time = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        fileName = "%s%s_%s.mat" % (suffix,fileName,str_time)
        fullName = "%s/%s" % (dirName,fileName)
        print "result saved to file: %s" %fileName
        print "and directory: %s" %dirName
        sio.savemat(fullName,saveDict)
    
    def savePlot(self,dirName='',fileName='',suffix='Plot_',ext='.tex'):
        if not dirName:
            dirName = os.path.split(os.path.abspath(os.path.realpath(sys.argv[0])))[0] + '/plots'
        if not os.path.isdir(dirName):
            os.mkdir(dirName)
        if not fileName:
            fileName = os.path.basename(main.__file__)
        if ".py" in fileName:
            fileName, file_ext = os.path.splitext(fileName)
        str_time = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        fileName = "%s/%s%s_%s%s" % (dirName,suffix,fileName,str_time,ext)
        matplotlib2tikz.save(fileName)

    def plot_q(self,show=True,save=False,title=True,grid=True,legend_str='',block=True,lb=[],ub=[],limits=False,Tvec=[],nj_plot=None):
        if len(Tvec) == 0:
            Tvec = self.T
        
        fig, ax = plt.subplots()
        # ax.plot(self.T,self.q)
        ax.plot(Tvec,self.q)

        if limits:
            if len(lb)==self.nj:
                for i in range(self.nj):
                    # ax.plot(self.T[[0,-1]],[ub[i],ub[i]], color=colors[i], linestyle='dashed')
                    # ax.plot(self.T[[0,-1]],[lb[i],lb[i]], color=colors[i], linestyle='dashed')
                    ax.plot(Tvec[[0,-1]],[ub[i],ub[i]], color=colors[i], linestyle='dashed')
                    ax.plot(Tvec[[0,-1]],[lb[i],lb[i]], color=colors[i], linestyle='dashed')
            elif len(lb)==nj_plot:
                for i in range(nj_plot):
                    ax.plot(Tvec[[0,-1]],[ub[i],ub[i]], color=colors[i], linestyle='dashed')
                    ax.plot(Tvec[[0,-1]],[lb[i],lb[i]], color=colors[i], linestyle='dashed')

        ax.set_xlabel("time [s]")
        ax.set_ylabel("$\\theta$ [rad]")
        if title:
            ax.set_title("Angular joint positions")
        ax.grid(grid)
        if not legend_str:
            legend_str = []
            for i in range(self.nj):
                legend_str += ["$q_{%s}$" % (i+1)]
        ax.legend(legend_str)
        ax.set_xlim(0,Tvec[-1])
        if show:
            plt.show(block)
        if save:
            self.savePlot(suffix='Plot_q_')

    def plot_qdot(self,show=True,save=False,title=True,grid=True,legend_str='',block=True,lb=[],ub=[],limits=False,Tvec=[],nj_plot=None):
        if len(Tvec) == 0:
            Tvec = self.T
        
        fig, ax = plt.subplots()
        # ax.plot(self.T,self.qdot)
        ax.plot(Tvec,self.qdot)

        if limits:
            if len(lb)==self.nj:
                for i in range(self.nj):
                    # ax.plot(self.T[[0,-1]],[ub[i],ub[i]], color=colors[i], linestyle='dashed')
                    # ax.plot(self.T[[0,-1]],[lb[i],lb[i]], color=colors[i], linestyle='dashed')
                    ax.plot(Tvec[[0,-1]],[ub[i],ub[i]], color=colors[i], linestyle='dashed')
                    ax.plot(Tvec[[0,-1]],[lb[i],lb[i]], color=colors[i], linestyle='dashed')
            elif len(lb)==nj_plot:
                for i in range(nj_plot):
                    ax.plot(Tvec[[0,-1]],[ub[i],ub[i]], color=colors[i], linestyle='dashed')
                    ax.plot(Tvec[[0,-1]],[lb[i],lb[i]], color=colors[i], linestyle='dashed')

        ax.set_xlabel("time [s]")
        ax.set_ylabel("$\\dot{\\theta}$ [rad/s]")
        if title:
            ax.set_title("Angular joint velocities")
        ax.grid(grid)
        if not legend_str:
            legend_str = []
            for i in range(self.nj):
                legend_str += ["$\\dot{q}_{%s}$" % (i+1)]
        ax.legend(legend_str)
        ax.set_xlim(0,Tvec[-1])
        if show:
            plt.show(block)
        if save:
            self.savePlot(suffix='Plot_qdot_')

    def plot_qddot(self,show=True,save=False,title=True,grid=True,legend_str='',block=True,lb=[],ub=[],limits=False,Tvec=[],nj_plot=None):
        if len(Tvec) == 0:
            Tvec = self.T

        fig, ax = plt.subplots()
        # ax.plot(self.T,self.qddot)
        ax.plot(Tvec,self.qddot)

        if limits:
            if len(lb)==self.nj:
                for i in range(self.nj):
                    # ax.plot(self.T[[0,-1]],[ub[i],ub[i]], color=colors[i], linestyle='dashed')
                    # ax.plot(self.T[[0,-1]],[lb[i],lb[i]], color=colors[i], linestyle='dashed')
                    ax.plot(Tvec[[0,-1]],[ub[i],ub[i]], color=colors[i], linestyle='dashed')
                    ax.plot(Tvec[[0,-1]],[lb[i],lb[i]], color=colors[i], linestyle='dashed')
            elif len(lb)==nj_plot:
                for i in range(nj_plot):
                    ax.plot(Tvec[[0,-1]],[ub[i],ub[i]], color=colors[i], linestyle='dashed')
                    ax.plot(Tvec[[0,-1]],[lb[i],lb[i]], color=colors[i], linestyle='dashed')

        ax.set_xlabel("time [s]")
        ax.set_ylabel("$\\ddot{\\theta}$ [rad/s$^2$]")
        if title:
            ax.set_title("Angular joint accelerations")
        ax.grid(grid)
        if not legend_str:
            legend_str = []
            for i in range(self.nj):
                legend_str += ["$\\ddot{q}_{%s}$" % (i+1)]
        ax.legend(legend_str)
        ax.set_xlim(0,Tvec[-1])
        if show:
            plt.show(block)
        if save:
            self.savePlot(suffix='Plot_qddot_')

    def plot_tau(self,show=True,save=False,title=True,grid=True,legend_str='',block=True,lb=[],ub=[],limits=False,Tvec=[],nj_plot=None):
        if len(Tvec) == 0:
            Tvec = self.T
        
        fig, ax = plt.subplots()
        # tau_plot = np.vstack((self.tau,self.tau[-1,:]))
        tau_plot = self.tau
        # print np.shape(tau_plot)
        # tau_plot[0,:] = DM.nan(1,self.nj).full()
        # ax.step(self.T,tau_plot)
        # ax.step(self.T,self.tau)
        ax.step(Tvec,self.tau, where='post')
        # ax.step(Tvec,self.tau)
        if limits:
            # for i in range(self.nj):
            #     ax.step(Tvec,ub[:,i], where='post', color=colors[i], linestyle='dashed')
            #     ax.step(Tvec,lb[:,i], where='post', color=colors[i], linestyle='dashed')
            if np.shape(lb)[1]==self.nj:
                for i in range(self.nj):
                    ax.step(Tvec,ub[:,i], where='post', color=colors[i], linestyle='dashed')
                    ax.step(Tvec,lb[:,i], where='post', color=colors[i], linestyle='dashed')
            elif np.shape(lb)[1]==nj_plot:
                for i in range(nj_plot):
                    ax.step(Tvec,ub[:,i], where='post', color=colors[i], linestyle='dashed')
                    ax.step(Tvec,lb[:,i], where='post', color=colors[i], linestyle='dashed')

        ax.set_xlabel("time [s]")
        ax.set_ylabel("$\\tau$ [Nm]")
        if title:
            ax.set_title("Joint effort")
        ax.grid(grid)
        if not legend_str:
            legend_str = []
            for i in range(self.nj):
                legend_str += ["$\\tau_{%s}$" % (i+1)]
        ax.legend(legend_str)
        ax.set_xlim(0,Tvec[-1])
        if show:
            plt.show(block)
        if save:
            self.savePlot(suffix='Plot_tau_')
    
    def plot_joint(self,joint=1,nq=1,show=True,save=False,title=True,grid=True,legend_str='',block=True,lb=[],ub=[],limits=False):
        index = joint - 1
        title_str = "Joint %s states" % joint

        fig, ax = plt.subplots()
        if nq >= 1:
            ax.plot(self.T,self.q[:,index])
        if nq >= 2:
            ax.plot(self.T,self.qdot[:,index])
        if nq >= 3:
            ax.plot(self.T,self.qddot[:,index])

        ax.set_xlabel("time [s]")
        if title:
            ax.set_title(title_str) 
        ax.grid(grid)
        if not legend_str:
            legend_str = []
            legend_str += ["$q_{%s}$" % joint]
            legend_str += ["$\\dot{q}_{%s}$" % joint]
            legend_str += ["$\\ddot{q}_{%s}$" % joint]
            legend_str = legend_str[:nq]
        ax.legend(legend_str)
        if show:
            plt.show(block)
        if save:
            self.savePlot(suffix='Plot_q_')

    def plot_trace(self):
        pass




###########################################
#   RANDOM FUNCTIONS
###########################################

def inSphere(point, ref, radius):
    if isinstance(point,list):
        point = np.asarray(point)
    if isinstance(ref,list):
        ref = np.asarray(ref)
    # Calculate the difference between the reference and measuring point
    diff = np.subtract(point, ref)
    # Calculate square length of vector (distance between ref and point)^2
    dist = np.sum(np.power(diff, 2))
    # If dist is less than radius^2, return True, else return False
    return dist < radius ** 2

def inWorkspace(point, ref, normal):
    if isinstance(point,list):
        point = np.asarray(point)
    if isinstance(ref,list):
        ref = np.asarray(ref)
    if isinstance(normal,list):
        normal = np.asarray(normal)
    # Calculate the difference between the reference and measuring point
    diff = np.subtract(point, ref)
    # If the scalar product between the normal and diff is greater than zero than the return value is True
    return np.inner(normal,diff) > 0
    