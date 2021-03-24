import random
from shapely.geometry.polygon import Polygon, LineString,Point
import copy
class obstacle():
    '''class for obstacles'''
    def __init__(self,nr_obs):
        self.nr_obs=nr_obs
        self.init_pos=[]
        self.pos=[]
        self.shape=[]
        self.vel=[]
        self.time=[]

    def add(self,init_pos,shape,vel):
        self.init_pos.append(init_pos)
        self.pos.append(copy.deepcopy(init_pos))
        self.shape.append(shape)
        self.vel.append(vel)
        self.time.append(0)
        
    def plot_obs(self):
        res=[]
        for i in range(self.nr_obs): #[x1,x2,x3,x4],[y1,y1,y2,y4]
            temp_x=[]
            temp_y=[]
            for index in range(4):
                temp_x.append(self.pos[i][0]+self.shape[i][index][0])
                temp_y.append(self.pos[i][1]+self.shape[i][index][1])
            res.append([temp_x,temp_y])
        return res

    def check_collision(self,curr_pos,next_pos):
        '''checking collision'''
        if next_pos[0]>10 or next_pos[0]<0 or next_pos[1]>10 or next_pos[1]<0:
            return False
        for i in range(self.nr_obs):
            line = LineString([(next_pos[0], next_pos[1]),(curr_pos[0], curr_pos[1])])
            points=[]
            for j in self.shape[i]:
                points.append((j[0]+self.pos[i][0],j[1]+self.pos[i][1]))
            polygon = Polygon(points)
            line1=LineString(points)
            #print(polygon.intersection(line))
            if polygon.intersection(line) or line1.intersection(line):
                return False
        return True
