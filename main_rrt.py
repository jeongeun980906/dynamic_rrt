from stage import Stage
from rrt import rrt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
import time

fiter=0 #just for trajectory visualization  
stage=Stage(10,10,2) #stage H, stage W, size of dynamic obstacle
RRT=rrt(stage) #rrt(stage) or rrt_star(stage)
traj=[[9,8],[1,7]] #just for debugging
done=False
obs_plot=None

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
        node1 = RRT.tree.pos[idx+i+1]
        if idx==0:
            ln=plt.plot(node1[0],node1[1],'b')
        else:
            node2 = RRT.tree.pos[parent_idx]
            ln=plt.plot([node1[0],node2[0]],[node1[1],node2[1]],'b', linewidth=0.2)
    if done==True:
        global ani
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
    ''''default setting for visuailization'''
    plt.axis([-4, 14, -4, 14])
    plt.plot(stage.pos_start[0],stage.pos_start[1],'ro')
    plt.plot(stage.pos_goal[0],stage.pos_goal[1],'ro')
    plt.grid()
    plt.fill([0,0,10,10], [0,10,10,0], edgecolor='r', fill=False)

def plot_traj():
    '''draw trajectory'''
    global fiter
    if fiter==0:
        plt.plot(traj[0][0],traj[1][0],'ro')
        fiter+=1
    elif len(traj[0]) > fiter:
        plt.plot(traj[0][fiter-1],traj[1][fiter-1],'ro') 
        plt.plot(traj[0][0:fiter],traj[1][0:fiter],'b')
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
    plt.show()
    print(':)')
    # ani.event_source.start()
    # ani.save('./result/tree.gif',writer=writergif)
    # ani.event_source.stop()
    # print('success')
    traj=RRT.backward()
    fig2 = plt.gcf()
    fig2.set_size_inches((18, 18))
    set_fig()
    traj=list(map(list, zip(*traj)))
    i=[i for i in range(150)]
    ani2 = FuncAnimation(fig2, update, i ,repeat=False,cache_frame_data=False)
    plt.show()