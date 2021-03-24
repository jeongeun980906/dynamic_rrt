from stage import Stage
from rrt import rrt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
import time
from rrt_star import rrt_star

fiter=0
stage=Stage(10,10,2)
RRT=rrt_star(stage)
traj=[[9,8],[1,7]]

def plot_stage():
    res=stage.obstacle.plot_obs()
    color=['g','c','m','y']
    for i in range(stage.nr_obs):
        ln=plt.fill(res[i][0],res[i][1],color[i])
    return ln,

def default():
    global fiter
    plt.axis([-4, 14, -4, 14])
    plt.plot(stage.pos_start[0],stage.pos_start[1],'ro')
    plt.plot(stage.pos_goal[0],stage.pos_goal[1],'ro')
    plt.grid()
    plt.fill([0,0,10,10], [0,10,10,0], edgecolor='r', fill=False)
    if len(traj[0]) > fiter:
        plt.plot(traj[0][fiter],traj[1][fiter],'ro') 
        plt.plot(traj[0][0:fiter],traj[1][0:fiter],'b')
        fiter+=1
    else:
        plt.plot(traj[0][-1],traj[1][-1],'ro') 
        plt.plot(traj[0],traj[1],'b')

def update(frame):
    stage.move_obs(frame)
    plt.clf()
    default()
    ln=plot_stage()
    return ln

if __name__=="__main__":
    writergif = PillowWriter(fps=30)
    fig = plt.gcf()
    fig.set_size_inches((18, 18))
    done=False
    while not done:
        done=RRT.step()
    traj=RRT.backward()
    traj=list(map(list, zip(*traj)))
    i=[i for i in range(150)]
    ani = FuncAnimation(fig, update, i ,repeat=False,cache_frame_data=False)
    ani.save('./result/animation5.gif',writer=writergif)
    #plt.show()