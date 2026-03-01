import tkinter as tk
from tkinter import ttk, messagebox
import random
import heapq
import math
import time


# ============================================================
# GRID CLASS
# ============================================================
class Grid:
    def __init__(self, rows, cols, obstacle_density=0.3):
        self.rows = rows
        self.cols = cols
        self.start = None
        self.goal = None
        self.grid = [[0 for _ in range(cols)] for _ in range(rows)]
        self.generate_random_map(obstacle_density)
    
    def generate_random_map(self, density):
        for row in range(self.rows):
            for col in range(self.cols):
                if random.random() < density:
                    self.grid[row][col] = 1
                else:
                    self.grid[row][col] = 0
        if self.start:
            self.grid[self.start[0]][self.start[1]] = 0
        if self.goal:
            self.grid[self.goal[0]][self.goal[1]] = 0
    
    def toggle_cell(self, row, col):
        if (row, col) == self.start or (row, col) == self.goal:
            return
        self.grid[row][col] = 1 - self.grid[row][col]
    
    def is_valid_position(self, row, col):
        return (0 <= row < self.rows and 
                0 <= col < self.cols and 
                self.grid[row][col] == 0)
# ============================================================
# HEURISTIC FUNCTIONS 
# ============================================================
def manhattan_distance(pos1, pos2):
    """Manhattan Distance: |x1-x2| + |y1-y2|"""
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

def euclidean_distance(pos1, pos2):
    """Euclidean Distance: √[(x1-x2)² + (y1-y2)²]"""
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

def get_heuristic(name):
    """Return the selected heuristic function"""
    if name == "Manhattan":
        return manhattan_distance
    else:  
        return euclidean_distance



# ============================================================
# A* ALGORITHM 
# ============================================================
def astar_search(grid, start, goal, heuristic_func):
   
    
    frontier = []
    counter = 0
    heapq.heappush(frontier, (heuristic_func(start, goal), counter, start))
    frontier_set = {start}
    
    came_from = {}
    
    g_scores = {start: 0}
    
    visited = set()
    frontier_nodes = {start}
    
    
    nodes_visited = 0
    start_time = time.time()
    
    while frontier:
        current = heapq.heappop(frontier)[2]
        frontier_set.remove(current)
        frontier_nodes.remove(current)
        
        if current in visited:
            continue
        
        visited.add(current)
        nodes_visited += 1
        
        if current == goal:
            path = []
            while current != start:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            
            time_taken = (time.time() - start_time) * 1000
            return path, visited, nodes_visited, len(path)-1, time_taken, frontier_nodes
        
        neighbors = [
            (current[0] - 1, current[1]), 
            (current[0] + 1, current[1]), 
            (current[0], current[1] - 1), 
            (current[0], current[1] + 1)   
        ]
        
        for neighbor in neighbors:
            if not grid.is_valid_position(neighbor[0], neighbor[1]):
                continue
            
            tentative_g = g_scores[current] + 1
            
            if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                came_from[neighbor] = current
                g_scores[neighbor] = tentative_g
                
                f_score = tentative_g + heuristic_func(neighbor, goal)
                
                if neighbor not in frontier_set:
                    counter += 1
                    heapq.heappush(frontier, (f_score, counter, neighbor))
                    frontier_set.add(neighbor)
                    frontier_nodes.add(neighbor)
    
    time_taken = (time.time() - start_time) * 1000
    return None, visited, nodes_visited, 0, time_taken, frontier_nodes

# ============================================================
# GREEDY BFS ALGORITHM 
# ============================================================
def greedy_bfs_search(grid, start, goal, heuristic_func):
   
    
    frontier = []
    counter = 0
    heapq.heappush(frontier, (heuristic_func(start, goal), counter, start))
    frontier_set = {start}
    
    came_from = {}
    
    visited = set()
    frontier_nodes = {start}
    
    nodes_visited = 0
    start_time = time.time()
    
    while frontier:
        current = heapq.heappop(frontier)[2]
        frontier_set.remove(current)
        frontier_nodes.remove(current)
        
        if current in visited:
            continue
        
        visited.add(current)
        nodes_visited += 1
        
        if current == goal:
            path = []
            while current != start:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            
            time_taken = (time.time() - start_time) * 1000
            return path, visited, nodes_visited, len(path)-1, time_taken, frontier_nodes
        
        neighbors = [
            (current[0] - 1, current[1]), 
            (current[0] + 1, current[1]), 
            (current[0], current[1] - 1),  
            (current[0], current[1] + 1)   
        ]
        
        for neighbor in neighbors:
            if not grid.is_valid_position(neighbor[0], neighbor[1]):
                continue
            
            if neighbor not in visited and neighbor not in frontier_set:
                came_from[neighbor] = current
                h_score = heuristic_func(neighbor, goal)
                counter += 1
                heapq.heappush(frontier, (h_score, counter, neighbor))
                frontier_set.add(neighbor)
                frontier_nodes.add(neighbor)
    
    time_taken = (time.time() - start_time) * 1000
    return None, visited, nodes_visited, 0, time_taken, frontier_nodes


