#!/usr/bin/env python

from radarGuidance import *
from wallFollower import *

import time
import random #used for the random choice of a strategy
import sys
import numpy as np
import math


#--------------------------------------
# Position of the goal:
goalx = 300
goaly = 450
# Initial position of the robot:
initx = 300
inity = 35
# strategy choice related stuff:
choice = -1
choice_tm1 = -1
tLastChoice = 0
rew = 0
t = time.time()

TIMESTEP = 0.001

i2name=['wallFollower','radarGuidance']

# Parameters of State building:
# threshold for wall consideration
th_neglectedWall = 35
# threshold to consider that we are too close to a wall
# and a punishment should be delivered
th_obstacleTooClose = 13
# angular limits used to define states
angleLMin = 0
angleLMax = 55

angleFMin=56
angleFMax=143

angleRMin=144
angleRMax=199

# Q-learning related stuff:
# definition of states at time t and t+1
S_t = ''
S_tm1 = ''
Q = {}
alpha = 0.8
gamma = 0.95
beta = 4

def pickInQ(s,a):
  global Q
  if (s,a) in Q:
    return Q[(s,a)]
  else:
    #Q[(s,a)] = 0
    return 0


#--------------------------------------
# the function that selects which controller (radarGuidance or wallFollower) to use
# sets the global variable "choice" to 0 (wallFollower) or 1 (radarGuidance)
# * arbitrationMethod: how to select? 'random','randPersist','qlearning'
def randomArbitration():
    global choice;
    choice = random.randrange(2);
def randomPersist():
    global timeLastChoix;
    global choice;
    if time.time() - timeLastChoix >= 2.0:
        timeLastChoix = time.time()
        choice = random.randrange(2)
def QlearningArbitration():
    global timeLastChoix;
    global ChoixABouclePrecedente
    global choice;
    global choice_tm1;
    global Q
    global rew
    global S_t
    global S_tm1
    global delta
    global alpha
    global gamma
    
    changementEtat = False;
    if S_t != S_tm1 :
      changementEtat = True;
      
    timeOut = False;
    if time.time() - timeLastChoix >= 2.0:
      timeOut = True;
  
    # =================== MAJ de Q(s,a) ===============================
    # mise à jour de Q(s,a) lorsque 
    # ou----> l'état vient de changer;
    # ou----> la récompense est non-nulle.
    # ou----> on a fait un choix à la boucle précédente     variab: choix==true
    if changementEtat or rew != 0 or ChoixABouclePrecedente:
      delta = rew + gamma * max([pickInQ(S_t, c) for c in [0, 1]]) - pickInQ(S_tm1, choice_tm1)
      Q[(S_tm1, choice_tm1)] =  pickInQ(S_tm1, choice_tm1) + alpha * delta

    # ============== Choix d'action =============================
    # choisir une nouvelle action lorsque 
    # ou----> l'état vient de changer 
    # ou----> la récompense est non-nulle 
    # ou----> il n'y a pas eu de changement de choix depuis 2 secondes. 
    if changementEtat or rew != 0 or timeOut:
        p = math.exp(beta * pickInQ(S_t, 0)) / sum([math.exp(beta * pickInQ(S_t, a)) for a in (0, 1)])
        choice_tm1 = choice;
        if random.uniform(0,1) < p:
            choice = 0
        else :
            choice = 1
        # MAJ de timeLastChoix ssi le choix est different du choix dernier
        if choice != choice_tm1:
            timeLastChoix = time.time();
        ChoixABouclePrecedente = True;
    else:
        ChoixABouclePrecedente = False;
        
def strategyGating(arbitrationMethod,verbose=True):
  global choice
  global choice_tm1
  global tLastChoice
  global rew
  global S_t
  global S_tm1
  global Q
  global alpha
  global gamma
  
  ###
  unkownStr = True;
  ###
  # The chosen gating strategy is to be coded here:
  #------------------------------------------------
  if arbitrationMethod=='random':
    unkownStr = False
    randomArbitration();
  #------------------------------------------------
  
  if arbitrationMethod=='randomPersist':
    unkownStr = False
    randomPersist();
  #------------------------------------------------
  if arbitrationMethod=='qlearning':
    unkownStr = False
    QlearningArbitration();
  #------------------------------------------------
  if unkownStr == True:
    print(arbitrationMethod+' unknown.')
    exit()

  if verbose:
    print("strategyGating: Active Module: "+i2name[choice])


#--------------------------------------
def buildStateFromSensors(laserRanges,radar,dist2goal):
  S   = ''
  # determine if obstacle on the left:
  wall='0'
  if min(laserRanges[angleLMin:angleLMax]) < th_neglectedWall:
    wall ='1'
  S += wall
  # determine if obstacle in front:
  wall='0'
  if min(laserRanges[angleFMin:angleFMax]) < th_neglectedWall:
    wall ='1'
    #print("Mur Devant")
  S += wall
  # determine if obstacle on the right:
  wall='0'
  if min(laserRanges[angleRMin:angleRMax]) < th_neglectedWall:
    wall ='1'
  S += wall

  S += str(radar)

  if dist2goal < 125:
    S+='0'
  elif dist2goal < 250:
    S+='1'
  else:
    S+='2'
  #print('buildStateFromSensors: State: '+S)

  return S

#--------------------------------------
g = 0;
def main(run):
  global g;
  global choice
  global choice_tm1
  global tLastChoice
  global Q
  global S_t
  global S_tm1
  global rew
  global timeLastChoix
  global ChoixABouclePrecedente
  timeLastChoix= time.time();
  ChoixABouclePrecedente = True;
  
  settings = Settings('worlds/entonnoir.xml')

  env_map = settings.map()
  robot = settings.robot()

  d = Display(env_map, robot)

  method = 'qlearning'
  # experiment related stuff
  startT = time.time()
  trial = 0
  nbTrials = 50
  trialDuration = np.zeros((nbTrials))
  trialLen = np.zeros((nbTrials))
  trialBings = np.zeros((nbTrials))
  
  ###-------------- Exercice4 --------------<##
  t0 = time.time();
  positions = [[]];
  positions_by_step = [[]]
  ############################################
  
  i = 0
  #####---------------- ##################
  experienceTotal = 10;
  nbstep = 0
  nbBing = 0

  ######################################
  while trial<nbTrials:  
    # ========================================== REWARD ==========================================
    rew = 0


    # update the display
    #-------------------------------------
    d.update()
    # get position data from the simulation
    #-------------------------------------
    pos = robot.get_pos()
    if time.time() - t0 > TIMESTEP * 10:
      positions[trial].append((pos.x(),pos.y()));
      t0 = time.time();
    positions_by_step[trial].append((pos.x(),pos.y()))
    # print("##########\nStep "+str(i)+" robot pos: x = "+str(int(pos.x()))+" y = "+str(int(pos.y()))+" theta = "+str(int(pos.theta()/math.pi*180.)))
    
    
    
    # has the robot found the reward ?
    #------------------------------------
    dist2goal = math.sqrt((pos.x()-goalx)**2+(pos.y()-goaly)**2)
    # if so, teleport it to initial position, store trial duration, set reward to 1:
    if (dist2goal<20): # 30
      #print('***** REWARD REACHED *****')
      pos.set_x(initx)
      pos.set_y(inity)
      robot.set_pos(pos) # format ?
      # and store information about the duration of the finishing trial:
      currT = time.time()
      trialDuration[trial] = currT - startT
      startT = currT
      trialLen[trial] = nbstep
      trialBings[trial] = nbBing
      print("Trial "+str(trial)+" duration:"+str(trialDuration[trial])+" nbstep:"+str(nbstep)+" nbbings:"+str(nbBing))
      trial +=1
      nbstep = 0
      nbBing = 0
      
      #####--------------- Exercice 4==================!!@####
      positions.append([]);
      positions_by_step.append([]);
      ############################################
      
      rew = 10

    # get the sensor inputs:
    #------------------------------------
    lasers = robot.get_laser_scanners()[0].get_lasers()
    laserRanges = []
    for l in lasers:
      laserRanges.append(l.get_dist())

    radar = robot.get_radars()[0].get_activated_slice()

    bumperL = robot.get_left_bumper()
    bumperR = robot.get_right_bumper()


    # 2) has the robot bumped into a wall ?
    #------------------------------------
    if bumperR or bumperL or min(laserRanges[angleFMin:angleFMax]) < th_obstacleTooClose:
      rew = -1
      nbBing += 1
      #print("***** BING! ***** "+i2name[choice])

    # 3) build the state, that will be used by learning, from the sensory data
    #------------------------------------
    S_tm1 = S_t
    choice_tm1 = choice   
    S_t = buildStateFromSensors(laserRanges,radar, dist2goal)

    #------------------------------------
    strategyGating(method,verbose=False)
    if choice==0:
      v = wallFollower(laserRanges,verbose=False)
    else:
      v = radarGuidance(laserRanges,bumperL,bumperR,radar,verbose=False)

    i+=1
    nbstep += 1 

    robot.move(v[0], v[1], env_map)
    time.sleep(TIMESTEP)

  # When the experiment is over:
  np.savetxt('log/'+str(run)+'_experience_tiralDuration'+method+'.txt',trialDuration)
  np.savetxt('log/'+str(run)+'_experience_tiralLen'+method+'.txt',trialLen)
  np.savetxt('log/'+str(run)+'_experience_tiralBings'+method+'.txt',trialBings)
  
  #=================Exercice 4=================================#
  import pickle
  with open('log/'+str(run)+'_experience_positions.pickle', 'wb') as handle:
    pickle.dump(positions, handle, protocol=pickle.HIGHEST_PROTOCOL)
  with open('log/'+str(run)+'_experience_positions_by_step.pickle', 'wb') as handle:
    pickle.dump(positions_by_step, handle, protocol=pickle.HIGHEST_PROTOCOL)
  with open('log/'+str(run)+'_experience_Q.pickle', 'wb') as handle:
    pickle.dump(Q, handle, protocol=pickle.HIGHEST_PROTOCOL)   
    # Load data (deserialize)
#with open('filename.pickle', 'rb') as handle:
#    unserialized_data = pickle.load(handle)
    
    
    
#--------------------------------------

if __name__ == '__main__':
  random.seed()
  main(64)
