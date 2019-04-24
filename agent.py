import numpy as np
from math import sqrt
import math

class Agent(object):

    def __init__(self, csvParameters, ksi=0.5, dhor = 10, timehor=5, goalRadiusSq=1, maxF = 10):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq =goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.dhor = dhor # the sensing radius
        self.timehor = timehor # the time horizon for computing avoidance forces
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = maxF # the maximum force that can be applied to the agent

    def computeForces(self, neighbors=[]):
        """ 
            Your code to compute the forces acting on the agent. 
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors
        """       
        
        infinity = math.inf
        epsilon = 0.2
        if not self.atGoal:
            Fgoal = (self.gvel - self.vel)/self.ksi
            #print('\n')
            for neigh in neighbors:
                distance = sqrt(pow(neigh.pos[0]-self.pos[0],2)+pow(neigh.pos[1]-self.pos[1],2))-self.radius-neigh.radius
                #print('Self Agent = ', self.id, 'Neighbor Agent = ', neigh.id )
                #print('distance = ', distance)
                if self.id == neigh.id:
                    continue
                elif sqrt(pow(neigh.pos[0]-self.pos[0],2)+pow(neigh.pos[1]-self.pos[1],2))-self.radius-neigh.radius <= 10:
                    tau = time_to_collision(self, neigh, epsilon)
                    #print('Time To Collision = ',tau)
                    if (tau != infinity) & (tau != 0):
                        Top = ((neigh.pos-self.pos)+((neigh.vel-self.vel)*tau))
                        Bottom = sqrt(pow(Top[0],2)+pow(Top[1],2))
                        n = Top/Bottom
                        Frepulsive = (max(self.timehor-tau,0)/tau)*n
                        Fgoal = Fgoal - Frepulsive
            
            if abs(Fgoal[0]) > self.maxF:
                if Fgoal[0] > 0:
                    Fgoal[0] = self.maxF
                elif Fgoal[0] < 0:
                    Fgoal[0] = self.maxF*-1

            if abs(Fgoal[1]) > self.maxF:
                if Fgoal[1] > 0:
                    Fgoal[1] = self.maxF
                elif Fgoal[1] < 0:
                    Fgoal[1] = self.maxF*-1
            
            #print('Fgoal = ', Fgoal)
            self.F = Fgoal
            

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            
            #print('\n Update function, self.id = ', self.id)
            self.vel += self.F*dt     # update the velocity
            if abs(self.vel[0]) > self.maxspeed:
                if self.vel[0] > 0:
                    self.vel[0] = self.maxspeed
                elif self.vel[0] < 0:
                    self.vel[0] = self.maxspeed*-1
            
            if abs(self.vel[1]) > self.maxspeed:
                if self.vel[1] > 0:
                    self.vel[1] = self.maxspeed
                elif self.vel[1] < 0:
                    self.vel[1] = self.maxspeed*-1
            
            #print('self.vel = ', self.vel)
            
            self.pos += self.vel*dt   #update the position
            #print('self.pos = ', self.pos)
            
            # compute the goal velocity for the next time step. do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed  
                

def time_to_collision(self, neigh, epsilon):
    infinity = math.inf
    rad = self.radius + neigh.radius
    w = self.pos - neigh.pos
    c = np.dot(w, w) - rad * rad
    if c< 0 :
        return 0
    v = self.vel - neigh.vel
    a = np.dot(v,v) - epsilon*epsilon
    b = np.dot(w,v) - epsilon*rad
    discr = b*b - a*c
    if b>0:
        return infinity
    if discr <= 0:
        return infinity
    tau = c/(-b + sqrt(discr))
    if tau < 0:
        return infinity
    return tau 
            
            
  