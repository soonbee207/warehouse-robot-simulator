# ============================================================
# FULL WAREHOUSE ROBOT SIMULATION (FIXED)
# 3 Robots + Collision Avoidance + Velocity Display
# R3 correctly moves from (0,0) to T4
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython.display import HTML
import heapq, math


# ============================================================
# 1. GRID WORLD
# ============================================================
class GridWorld:
    def __init__(self, width=10, height=10, obstacles=None):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width))

        if obstacles:
            for (x, y) in obstacles:
                self.grid[y][x] = 1

    def is_free(self, x, y):
        return (
            0 <= x < self.width and 
            0 <= y < self.height and 
            self.grid[y][x] == 0
        )


# ============================================================
# 2. A* PATH PLANNER
# ============================================================
class AStarPlanner:
    def __init__(self, grid):
        self.grid = grid

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node):
        x, y = node
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = x+dx, y+dy
            if self.grid.is_free(nx, ny):
                yield (nx, ny)

    def plan(self, start, goal):
        if goal is None:
            return []
        open_list=[(self.heuristic(start,goal),0,start,[start])]
        visited=set()
        while open_list:
            est,cost,node,path=heapq.heappop(open_list)
            if node==goal:
                return path
            if node in visited:
                continue
            visited.add(node)
            for nbr in self.get_neighbors(node):
                if nbr not in visited:
                    nc=cost+1
                    est = nc + self.heuristic(nbr,goal)
                    heapq.heappush(open_list,(est,nc,nbr,path+[nbr]))
        return []


# ============================================================
# 3. ROBOT CLASS
# ============================================================
class Robot:
    def __init__(self, name, start, planner, color):
        self.name=name
        self.position=np.array(start,dtype=float)
        self.planner=planner
        self.color=color

        self.goal=None
        self.path=[]
        self.step=0

        self.v_max=1.0
        self.v=1.0
        self.prev_v=0.0
        self.acc=0.0

        self.wait=False
        self.carrying=None

    def distance_to(self,other):
        return np.linalg.norm(self.position-other.position)

    def assign_goal(self,goal):
        self.goal=goal
        if goal:
            self.path=np.array(self.planner.plan(tuple(self.position.astype(int)),goal),dtype=float)
        else:
            self.path=[]
        self.step=0

    def move(self, robots):
        if self.goal is None or len(self.path)==0 or self.step>=len(self.path)-1:
            self.v=0
            self.acc=self.v-self.prev_v
            self.prev_v=self.v
            return

        if self.wait:
            self.v=0
            self.acc=self.v-self.prev_v
            self.prev_v=self.v
            return

        min_d=min([self.distance_to(r) for r in robots if r!=self]+[999])
        if min_d>2:
            self.v=self.v_max
        else:
            self.v=max(0.1,self.v_max*np.exp(-1.2*(2-min_d)))

        target=self.path[self.step+1]
        dvec=target-self.position
        dist=np.linalg.norm(dvec)

        if dist<1e-3:
            self.step+=1
            return

        step=min(self.v,dist)
        self.position+=(dvec/dist)*step

        self.acc=self.v-self.prev_v
        self.prev_v=self.v


# ============================================================
# 4. CONFLICT DETECTION
# ============================================================
def predict_conflict(r1,r2):
    if len(r1.path)==0 or len(r2.path)==0:
        return False
    c1=tuple(np.floor(r1.position).astype(int))
    c2=tuple(np.floor(r2.position).astype(int))
    if c1==c2:
        return True
    n1=r1.path[min(r1.step+1,len(r1.path)-1)]
    n2=r2.path[min(r2.step+1,len(r2.path)-1)]
    if np.allclose(n1,n2,atol=0.1):
        return True
    if np.allclose(n1,r2.position,atol=0.1) and np.allclose(n2,r1.position,atol=0.1):
        return True
    if np.linalg.norm(r1.position-r2.position)<1.0:
        return True
    return False

def compute_priority(r1,r2):
    if r1.carrying and not r2.carrying:
        return r1
    if r2.carrying and not r1.carrying:
        return r2
    d1 = np.linalg.norm(r1.position-np.array(r1.goal)) if r1.goal else 999
    d2 = np.linalg.norm(r2.position-np.array(r2.goal)) if r2.goal else 999
    return r1 if d1<d2 else r2


# ============================================================
# 5. SIMULATION
# ============================================================
def simulate():

    obstacles=[(2,2),(2,7),(7,2),(7,7)]
    env=GridWorld(10,10,obstacles)
    planner=AStarPlanner(env)

    tasks=[(3,3),(6,3),(3,6),(6,6)]  # T1 T2 T3 T4

    drop_R1=(0,9)
    drop_R2=(9,0)
    drop_R3=(9,9)

    r1=Robot("R1",(0,5),planner,"dodgerblue")
    r2=Robot("R2",(5,0),planner,"orange")
    r3=Robot("R3",(0,0),planner,"green")

    robots=[r1,r2,r3]

    # CORRECT ASSIGNMENT
    r1.assign_goal(tasks[0])  # T1
    r2.assign_goal(tasks[2])  # T3
    r3.assign_goal(tasks[3])  # T4 ← FIXED
    
    # DEBUG: Print paths
    print(f"R1 path length: {len(r1.path)}")
    print(f"R2 path length: {len(r2.path)}")
    print(f"R3 path length: {len(r3.path)}, path: {r3.path[:5] if len(r3.path)>0 else 'EMPTY'}")

    fig,ax=plt.subplots(figsize=(6,6))
    ax.set_xlim(-0.5,9.5)
    ax.set_ylim(-0.5,9.5)
    ax.set_xticks(range(10))
    ax.set_yticks(range(10))
    ax.grid(True)

    for (x,y) in obstacles:
        ax.add_patch(plt.Rectangle((x-0.5,y-0.5),1,1,color="saddlebrown"))
        ax.text(x,y,"S",color="white",ha="center",va="center")

    for i,t in enumerate(tasks):
        ax.add_patch(plt.Rectangle((t[0]-0.5,t[1]-0.5),1,1,color="yellow",alpha=0.4))
        ax.text(t[0],t[1],f"T{i+1}",ha="center")

    scat=[ax.plot([],[],'o',color=r.color)[0] for r in robots]
    labels=[ax.text(0,0,"",fontsize=8,color=r.color) for r in robots]
    box={r1:None,r2:None,r3:None}

    def update(frame):

        # Reset wait flags first
        for r in robots:
            r.wait=False

        # Check conflicts across ALL pairs
        pairs=[(r1,r2),(r1,r3),(r2,r3)]
        for a,b in pairs:
            if predict_conflict(a,b):
                winner=compute_priority(a,b)
                loser=b if winner is a else a
                loser.wait=True

        # Move robots
        for r in robots:
            r.move(robots)

        # Pickup
        for r in robots:
            if r.goal in tasks:
                if np.linalg.norm(r.position-np.array(r.goal))<0.2:
                    r.carrying=r.goal
                    if r.name=="R1":
                        r.assign_goal(drop_R1)
                    elif r.name=="R2":
                        r.assign_goal(drop_R2)
                    else:
                        r.assign_goal(drop_R3)

        # Dropoff
        for r in robots:
            if r.carrying:
                if np.linalg.norm(r.position-np.array(r.goal))<0.2:
                    r.carrying=None
                    r.assign_goal(None)

        # DRAW
        for i,r in enumerate(robots):
            scat[i].set_data([r.position[0]],[r.position[1]])
            labels[i].set_position((r.position[0],r.position[1]+0.3))
            labels[i].set_text(f"{r.name}\nv={r.v:.2f}\na={r.acc:.2f}")

            if r.carrying:
                if box[r] is None:
                    box[r]=ax.add_patch(
                        plt.Rectangle(
                            (r.position[0]-0.3,r.position[1]-0.3),
                            0.6,0.6,color="green"))
                else:
                    box[r].set_xy((r.position[0]-0.3,r.position[1]-0.3))
            else:
                if box[r]:
                    box[r].remove()
                    box[r]=None

        return scat + labels

    ani=FuncAnimation(fig,update,frames=300,interval=120,blit=True)
    display(HTML(ani.to_jshtml()))
    plt.close()

simulate()
