from stage import Stage
from rrt_star import rrt_star
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
import time
import math

vel=2
fiter=0 #just for trajectory visualization  
stage=Stage(10,10,vel) #stage H, stage W, size of dynamic obstacle
RRT=rrt_star(stage) #rrt(stage) or rrt_star(stage)
traj=[[9,8],[1,7]] #just for debugging
done=False
obs_plot=None
PI=math.pi

def plot_stage():
    '''draw moving obstacle'''
    res=stage.obstacle.plot_obs()
    color=['g','c','m','y']
    plt.fill(res[0][0],res[0][1],color[0])
    plt.fill(res[1][0],res[1][1],color[1])
    ln1=plt.fill(res[2][0],res[2][1],color[2])
    ln2=plt.fill(res[3][0],res[3][1],color[3])
    return ln1, ln2

def plot_tree(idx):
    '''visualize tree constructing'''
    done=RRT.step()
    if RRT.cnt==0:
        return None
    for i in range(RRT.cnt):
        parent_idx = RRT.tree.parent[idx+i+1] 
        node1 = RRT.tree.pos[idx+i+1] # next_pos
        if idx==0:
            ln=plt.plot(node1[0],node1[1],'b')
        else:
            #print(RRT.tree.changes)
            if RRT.tree.changes!=[]:
                for a in RRT.tree.changes:
                    temp1=RRT.tree.pos[a[0]] # near node
                    temp2=RRT.tree.pos[a[1]] # parent to be removed
                    temp3=RRT.tree.parent[a[0]]
                    temp4=RRT.tree.pos[temp3]
                    plt.plot([temp1[0],temp2[0]],[temp1[1],temp2[1]],'w', linewidth=0.2) # update rewiring visualization
                    plt.plot([temp4[0],temp1[0]],[temp4[1],temp1[1]],'b', linewidth=0.2)
            node2 = RRT.tree.pos[parent_idx]
            ln=plt.plot([node1[0],node2[0]],[node1[1],node2[1]],'b', linewidth=0.2) # update new node
    if done==True:
        global ani
        plt.show()
        ani.event_source.stop()
        print('please close the figure')
    return ln

def animate_tree(i):
    '''visualize tree constructing'''
    idx = RRT.nr_node
    frame = RRT.tree.time[idx]
    stage.move_obs(frame)
    global obs_plot
    #print(obs_plot)
    if obs_plot != None:
        obs_plot[0][0].set_visible(False)
        obs_plot[1][0].set_visible(False)
    ln1,ln2 = plot_stage()
    obs_plot=[ln1,ln2]
    ln = plot_tree(idx)
    return ln

def set_fig():
    '''default setting for figure'''
    plt.axis([-4, 14, -4, 14])
    plt.plot(stage.pos_start[0],stage.pos_start[1],'ro')
    plt.plot(stage.pos_goal[0],stage.pos_goal[1],'ro')
    plt.grid()
    plt.fill([0,0,10,10], [0,10,10,0], edgecolor='r', fill=False)

def plot_traj():
    '''ploting trajectory'''
    global fiter
    if fiter==0:
        plt.plot(traj[0][0],traj[1][0],'ro')
        fiter+=1
    elif len(traj[0]) > fiter:
        plt.plot(traj[0][fiter],traj[1][fiter],'ro') 
        plt.plot(traj[0][0:fiter+1],traj[1][0:fiter+1],'b')
        fiter+=1
    else:
        plt.plot(traj[0][-1],traj[1][-1],'ro') 
        plt.plot(traj[0],traj[1],'b')

def update(frame):
    '''animation visuailization'''
    stage.move_obs(frame)
    plt.clf()
    set_fig()
    plot_traj()
    ln=plot_stage()
    return ln

if __name__=="__main__":
    writergif = PillowWriter(fps=30)
    fig = plt.gcf()
    fig.set_size_inches((18, 18))
    set_fig()
    global ani
    ani=FuncAnimation(fig, animate_tree, repeat=False)
    for i in range(100):
        print('frame:',i)
        ani.save('./result/rrt_star_tree_'+str(vel)+str(i)+'.mp4',fps=30)
    #plt.show()
    print(':)')
    traj=RRT.backward()
    fig2 = plt.gcf()
    fig2.set_size_inches((18, 18))
    set_fig()
    traj=list(map(list, zip(*traj)))
    i=[i for i in range(150)]
    ani2 = FuncAnimation(fig2, update, i ,repeat=False,cache_frame_data=False)
    ani2.save('./result/rrt_star_'+str(vel)+'v1.gif',writer=writergif)
    #plt.show()