import random
import math
import numpy as np

PI=math.pi

class tree():
    def __init__(self,max_rrt):
        self.pos=[[0,0]]*max_rrt
        self.u=[0]*max_rrt
        self.child=[0]*max_rrt
        self.parent=[0]*max_rrt
        self.status=['Nan']*max_rrt
        self.color=['g']*max_rrt
        self.time=[0]*max_rrt

class rrt():
    def __init__(self,stage,max_rrt=1E4):
        self.stage=stage
        self.pos_goal=self.stage.pos_goal
        self.pos_start=self.stage.pos_start
        self.max_rrt=max_rrt
        self.tree=tree(int(max_rrt))
        self.nr_node=0
        self.color_pallate=['g','c','m','y']
        self.cnt_expansion=10
        self.vmin=1E-1
        self.vmax=5E-1
        self.threshold=1E-1
        self.iter=1
        self.maxIter=1E4
        self.tree.pos[0]=self.pos_start
        self.tree.u[0]=[0,0]
        self.tree.parent[0]=0
        self.tree.status[0]='root'
        self.tree.time[0]=0

    def add_node(self,pos,u,parent,status,time):
        self.nr_node+=1
        idx=self.nr_node
        self.tree.pos[idx]=pos
        self.tree.u[idx]=u
        self.tree.parent[idx]=parent
        self.tree.status[idx]=status
        self.tree.color[idx]=self.color_pallate[random.randint(0,3)]
        self.tree.time[idx]=time
    
    def expand_node(self,fr_idx,next_pos,u):
        time=self.tree.time[fr_idx]
        self.add_node(next_pos,u,fr_idx,'node',time+1)
    
    def search_shortest_node(self, pos, min_time=0):
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

    def step(self):
        #print(self.stage.obstacle.pos[2])
        rand_pos=[random.random()*self.stage.w, self.stage.h*random.random()]
        idx_closest = self.search_shortest_node(rand_pos)
        curr_pos = self.tree.pos[idx_closest]
        curr_time = self.tree.time[idx_closest]
        for i in range(self.cnt_expansion):
            rand_rad=2*PI*random.random()
            rand_vel=max(self.vmax*random.random(),self.vmin)
            rand_ux=rand_vel*math.cos(rand_rad)
            rand_uy=rand_vel*math.sin(rand_rad)
            rand_u=[rand_ux,rand_uy]
            next_pos=[curr_pos[0]+rand_u[0],curr_pos[1]+rand_u[1]]
            #print(curr_time)
            self.stage.move_obs(curr_time+1)
            if self.stage.obstacle.check_collision(curr_pos,next_pos):
                curr_next_dist=np.linalg.norm(np.asarray(rand_pos)-np.asarray(next_pos))
                self.expand_node(idx_closest,next_pos,rand_u)
            dist2goal=np.linalg.norm(np.asarray(next_pos)-np.asarray(self.pos_goal))
            if dist2goal<self.threshold:
                print("GOAL! distance to goal left {}".format(dist2goal))
                print("curr_time: {}".format(curr_time))
                return True

        return False
    
    def backward(self):
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