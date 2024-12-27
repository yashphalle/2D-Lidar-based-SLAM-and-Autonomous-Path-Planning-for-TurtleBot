import numpy as np
import math


def calc_cost(x1,y1,x2,y2):
    return math.sqrt((y2-y1)^2+(x2-x1)^2)

def calc_f(curr,goal,start):
    return calc_cost(curr[0],curr[1],goal[0],goal[1])+calc_cost(curr[0],curr[1],start[0],start[1])

def add_neighbours(grid,curr):
    if(grid[curr[0]-1],grid[curr[1]]!=1):
        hp.push([curr[0]-1,curr[1]],calc_f())
    if(grid[curr[0]-1],grid[curr[1]-1]!=1):
        hp.push([curr[0]-1,curr[1]-1])
    if(grid[curr[0]-1],grid[curr[1]+1]!=1):
        hp.push([curr[0]-1,curr[1]+1])
    if(grid[curr[0]],grid[curr[1]+1]!=1):
        hp.push([curr[0],curr[1]+1])
    if(grid[curr[0]],grid[curr[1]-1]!=1):
        hp.push([curr[0],curr[1]-1])
    if(grid[curr[0]+1],grid[curr[1]]!=1):
        hp.push([curr[0],curr[1]])
    if(grid[curr[0]+1],grid[curr[1]-1]!=1):
        hp.push([curr[0],curr[1]-1])
    if(grid[curr[0]+1],grid[curr[1]+1]!=1):
        hp.push([curr[0],curr[1]+1])


def a_star(grid,goal,start):
    heap hp
    hp.push(start)
    curr = start
    while (curr!=goal):
        curr= heap.pop()
        add_neighbours()


    print(grid)


if __name__ == "__main__":
    print("A* babyyy")
    grid = np.array([[0,0,0,0,0],
                    [0,0,0,0,0],
                    [0,0,0,0,0],
                    [0,0,0,0,0],
                    [0,0,0,0,0]])
    a_star(grid)

    

