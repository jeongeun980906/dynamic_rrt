import matplotlib.pyplot as plt
import numpy as np
from obstacle import obstacle
from matplotlib.animation import FuncAnimation
import copy

class Stage():
    def __init__(self,w,h,size,nr_obs=4):
        self.w=w
        self.h=h
        self.nr_obs=nr_obs
        self.obstacle=obstacle(nr_obs)
        self.set_obstacle(size)
        self.set_goal()
        
    def set_obstacle(self,size):
        self.obstacle.add([0,5], [[0,0],[3,0] ,[5 ,2] ,[0, 2]], [0, 0])
        self.obstacle.add([5,3],[[0,0],[5,0],[5,2],[2,2]],[0,0])
        self.obstacle.add([1,1],[[size,0],[0,size],[-size,0],[0,-size]],[0.1, 0.1])
        self.obstacle.add([9,9],[[size,0],[0,size],[-size,0],[0,-size]],[-0.1,-0.1])

    def set_goal(self):
        self.pos_start=[9,1]
        self.pos_goal=[1,9]

    def move_obs(self,frame):
        for i in range(self.nr_obs):
            self.obstacle.pos[i][0]=frame*self.obstacle.vel[i][0]+self.obstacle.init_pos[i][0]
            self.obstacle.pos[i][1]=self.obstacle.vel[i][1]*frame+self.obstacle.init_pos[i][1]