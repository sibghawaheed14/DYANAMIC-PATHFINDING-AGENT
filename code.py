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
