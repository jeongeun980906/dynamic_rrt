import random
import math
import numpy as np
import copy

PI=math.pi

class tree():
    def __init__(self,max_rrt):
        self.pos=[[0,0]]*max_rrt
        self.cost=[0]*max_rrt
        self.child=[0]*max_rrt
        self.parent=[0]*max_rrt
        self.status=['Nan']*max_rrt
        self.time=[0]*max_rrt

class rrt_star():
    '''rrt* algorithm'''
    def __init__(self,stage,max_rrt=1E5,radius=0.1):
        self.stage=stage
        self.pos_goal=self.stage.pos_goal
        self.pos_start=self.stage.pos_start
        self.max_rrt=max_rrt
        self.radius=radius
        self.tree=tree(int(max_rrt))
        self.nr_node=0

        # hyperparameters
        self.cnt_expansion=10
        self.vmin=1E-1
        self.vmax=5E-1
        self.threshold=1E-1
        self.iter=1
        self.maxIter=1E4
        
        # init
        self.tree.pos[0]=self.pos_start
        self.tree.cost[0]=0
        self.tree.parent[0]=0
        self.tree.status[0]='root'
        self.tree.time[0]=0

    def add_node(self,pos,cost,parent,status,time):
        '''add node in tree
        time is for dynamic env'''
        self.nr_node+=1
        idx=self.nr_node
        self.tree.pos[idx]=pos
        self.tree.cost[idx]=cost
        self.tree.parent[idx]=parent
        self.tree.status[idx]=status
        self.tree.time[idx]=time
    
    def expand_node(self,fr_idx,next_pos,cost):
        '''expand node in tree'''
        time=self.tree.time[fr_idx]
        self.add_node(next_pos,cost,fr_idx,'node',time+1)
    
    def search_shortest_node(self, pos, min_time=0):
        '''given random position in stage, find nearest node
        it becomes current pose for random search'''
        min_dist = math.inf
        min_idx  = 0
        for i in range(self.nr_node):
            curr_pos=self.tree.pos[i]
            curr_dist=np.linalg.norm(np.asarray(curr_pos)-np.asarray(pos))
            curr_time=self.tree.time[i]
            if (curr_dist < min_dist) and (curr_time >= min_time):
                min_dist=curr_dist
                min_idx=i
        return min_idx

    def near(self,pos):
        '''find nodes which is close to new node
        can tune this one by radius'''
        idx=[]
        for i in range(self.nr_node):
            curr_pos=self.tree.pos[i]
            curr_dist=np.linalg.norm(np.asarray(curr_pos)-np.asarray(pos))
            if (curr_dist < self.radius):
                idx.append(i)
        return idx

    def step(self):
        '''searching through stage...'''
        # select random pose
        rand_pos=[random.random()*self.stage.w, self.stage.h*random.random()]
        # get nearest node which becomes current position
        idx_closest = self.search_shortest_node(rand_pos)
        curr_pos = self.tree.pos[idx_closest]
        curr_time = self.tree.time[idx_closest]
        for i in range(self.cnt_expansion):
            # select next position based on velocity
            rand_rad=2*PI*random.random()
            rand_vel=max(self.vmax*random.random(),self.vmin)
            rand_ux=rand_vel*math.cos(rand_rad)
            rand_uy=rand_vel*math.sin(rand_rad)
            rand_u=[rand_ux,rand_uy]
            next_pos=[curr_pos[0]+rand_u[0],curr_pos[1]+rand_u[1]]
            # for simulation, move obstacles
            self.stage.move_obs(curr_time+1)
            if self.stage.obstacle.check_collision(curr_pos,next_pos):
                distance=np.linalg.norm(np.asarray(curr_pos)-np.asarray(next_pos))
                cmin=self.tree.cost[idx_closest]+distance
                xmin=idx_closest
                near=self.near(next_pos)
                for x in near: # connecting min path
                    c_pos=self.tree.pos[x]
                    if self.stage.obstacle.check_collision(c_pos,next_pos):
                        distance=np.linalg.norm(np.asarray(c_pos)-np.asarray(next_pos))
                        c=self.tree.cost[x]+distance
                        if c<cmin:
                            cmin=copy.deepcopy(c)
                            xmin=copy.deepcopy(x)
                next_pos_cost=cmin
                self.expand_node(xmin,next_pos,cmin)
                for x in near: # rewire
                    c_pos=self.tree.pos[x]
                    if self.stage.obstacle.check_collision(c_pos,next_pos):
                        distance=np.linalg.norm(np.asarray(c_pos)-np.asarray(next_pos))
                        c=next_pos_cost+distance
                        if c<self.tree.cost[x]:
                            if x != 0:
                                self.tree.parent[x]= copy.deepcopy(self.nr_node)#next index
            dist2goal=np.linalg.norm(np.asarray(next_pos)-np.asarray(self.pos_goal))
            if dist2goal<self.threshold:
                print("GOAL! distance to goal left {}".format(dist2goal))
                print("curr_time: {}".format(curr_time))
                return True
            self.iter+=1
            return False
    
    def backward(self):
        '''find optimal trajectory based on tree'''
        nr_node=self.nr_node
        final_time=self.tree.time[nr_node]
        parent_idx=self.tree.parent[nr_node]
        rrt_path=[[0,0]]*(final_time+1)
        rrt_path[0]=self.stage.pos_start
        rrt_path[-1]=self.tree.pos[nr_node]
        for i in range(final_time,1,-1):
            curr_pos=self.tree.pos[parent_idx]
            parent_idx=self.tree.parent[parent_idx]
            rrt_path[i-1]=curr_pos
            if self.tree.status=='root':
                break
        return rrt_path