# -*- coding: utf-8 -*-

#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia

#   This script is purely based on the optimal control framework
#   of CasADi toolbox. Without it the transcription will not work.
#   Make sure an adequate version of CasADi is installed,
#   version 3.5.0 is desired.

#   Functionality:
#   - Multi-stage
#   - Easy adding constraints
#   - Easy adding optimization variables
#   

from casadi import *
import numpy as np

class DirectTranscription:
    # This is the transcription class which takes as input the different 
    # optimization variables and constraints.
    def __init__(self):
        # OPTIMIZATION VARIABLES
        self.w = []
        self.w_0 = []
        self.w_lb = []
        self.w_ub = []
        self.w_set = {}
        self.w_set_counter = 0

        # INEQUALITY CONSTRAINTS
        self.g = []
        self.g_lb = []
        self.g_ub = []
        self.g_set = {}
        self.g_set_counter = 0

        # COST FUNCTIONS
        # Lagrange term: iterative cost function
        self.L_set = {}
        self.L_set_counter = 0
        # Mayer term: terminal cost function
        self.E_set = {}
        self.E_set_counter = 0

        # MULTI-STAGE NLP
        self.N_stage = 1
    
    def defineStages(num_stage=1):
        self.N_stage = num_stage

    def addVariable(**kwargs):
        # This fucntion gives the user the opportunity to add new optimization
        # variables to the NLP. Either of an entire stage or just at the beginning
        # of a stage.

        # Create class for a variable that keeps track of all indexers etc
        pass
        

    def addConstraint():
        # This function allows the user to add a constraint to g, either in- or
        # equality constraint depending on the inputs. The user can choose if 
        # it applied to a single stage or all, or if the constraint is an
        # initial constraint or terminal constraint. 
        pass

    def transcripe():
        # This function will be the "final" function because it will use direct
        # transcription to take all the input data of the class into a solvable
        # NLP.
        pass


class BaseVariable:
    # This will be the parent class of several subclasses: optimization variable,
    # (in)equality constraint, objective function, etc.
    def __init__(self,size=1,name='x'):
        self.str = name
        self.size = size

    def add


class OptVariable(BaseVariable):
    # This class is used to create Optimization variables
    
    def addInitialCondition(self):
        pass

class OptConstraint(BaseVariable):
    # This class is used to create Constraints
    pass

