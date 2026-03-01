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


# ============================================================
#  DYNAMIC OBSTACLES
# ============================================================
class SimpleDynamicObstacleManager:
    def __init__(self, grid):
        self.grid = grid
        self.counter = 0
        
    def update(self, current_path, agent_position):
        self.counter += 1
        
        if self.counter % 4 != 0:  
            return False
        
        if not agent_position:
            return False
        
        path_blocked = False
        print("Adding new obstacles...")  
        
        num_obstacles = random.randint(2, 3)
        for _ in range(num_obstacles):
            row = random.randint(0, self.grid.rows - 1)
            col = random.randint(0, self.grid.cols - 1)
            pos = (row, col)
            
            if (pos != self.grid.start and 
                pos != self.grid.goal and 
                pos != agent_position):
                
                if self.grid.grid[row][col] == 0:
                    self.grid.grid[row][col] = 1
                    print(f"  New obstacle at ({row}, {col})")
                    
                    if current_path and pos in current_path:
                        path_blocked = True
                        print(f"   Path blocked at ({row}, {col})!")
        
        return path_blocked

# ============================================================
#  GUI 
# ============================================================
class PathfindingGUI:
    def __init__(self, rows=15, cols=20):
        print("Starting PathfindingGUI...")
        self.rows = rows
        self.cols = cols
        self.cell_size = 25
        
        self.grid_width = cols * self.cell_size
        self.grid_height = rows * self.cell_size
        
        self.root = tk.Tk()
        self.root.title("Dynamic Pathfinding Agent")
        self.root.geometry(f"{self.grid_width + 40}x{self.grid_height + 280}")
        
        self.grid = Grid(rows, cols, obstacle_density=0.3)
        self.dynamic_manager = SimpleDynamicObstacleManager(self.grid)  
        
        self.COLORS = {
            'empty': 'white',
            'obstacle': 'black',
            'start': 'light green',
            'goal': 'light coral',
            'visited': 'light blue',
            'path': 'green',
            'frontier': 'yellow',
            'grid_line': 'gray'
        }
        
        self.agent_position = None
        self.current_path = []
        self.visited_nodes = set()
        self.frontier_nodes = set()
        
        self.nodes_visited = 0
        self.path_cost = 0
        self.execution_time = 0
        
        self.algorithm = tk.StringVar(value="A*")
        self.heuristic = tk.StringVar(value="Manhattan")
        self.dynamic_mode = tk.BooleanVar(value=False)
        
        self.status_message = ""
        
        self.create_widgets()
        self.draw_grid()
        
        self.update_dynamic()
        
        print(" PathfindingGUI ready!")
        print(f"  Default: {self.algorithm.get()} with {self.heuristic.get()} heuristic")
    
    def create_widgets(self):
        control_frame = ttk.LabelFrame(self.root, text="Controls", padding=5)
        control_frame.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(control_frame, text="Algorithm:").grid(row=0, column=0, padx=5)
        algo_combo = ttk.Combobox(control_frame, textvariable=self.algorithm, 
                                  values=["A*", "Greedy BFS"], width=12, state="readonly")
        algo_combo.grid(row=0, column=1, padx=5)
        algo_combo.bind('<<ComboboxSelected>>', self.on_setting_change)
        
        ttk.Label(control_frame, text="Heuristic:").grid(row=0, column=2, padx=5)
        heuristic_combo = ttk.Combobox(control_frame, textvariable=self.heuristic,
                                       values=["Manhattan", "Euclidean"], 
                                       width=12, state="readonly")
        heuristic_combo.grid(row=0, column=3, padx=5)
        heuristic_combo.bind('<<ComboboxSelected>>', self.on_setting_change)
        
        ttk.Checkbutton(control_frame, text="Dynamic Mode", 
                       variable=self.dynamic_mode).grid(row=0, column=4, padx=10)
        
        ttk.Button(control_frame, text="Find Path", 
                  command=self.find_path).grid(row=0, column=5, padx=5)
        ttk.Button(control_frame, text="Random Map", 
                  command=self.random_map).grid(row=0, column=6, padx=5)
        ttk.Button(control_frame, text="Clear", 
                  command=self.clear_all).grid(row=0, column=7, padx=5)
        
        settings_frame = ttk.Frame(self.root)
        settings_frame.pack(fill="x", padx=5, pady=2)
        
        self.settings_label = ttk.Label(settings_frame, text="", foreground="blue")
        self.settings_label.pack()
        self.update_settings_display()
        
        canvas_frame = ttk.Frame(self.root)
        canvas_frame.pack(padx=5, pady=5)
        
        self.canvas = tk.Canvas(canvas_frame, width=self.grid_width, 
                               height=self.grid_height, bg='white', highlightthickness=1,
                               highlightbackground='gray')
        self.canvas.pack()
        
        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<Shift-Button-1>", self.on_shift_click)
        self.canvas.bind("<Control-Button-1>", self.on_ctrl_click)
        
        metrics_frame = ttk.LabelFrame(self.root, text="Metrics", padding=5)
        metrics_frame.pack(fill="x", padx=5, pady=5)
        
        self.metrics_labels = {}
        
        ttk.Label(metrics_frame, text="Nodes Visited:").grid(row=0, column=0, padx=5)
        self.metrics_labels['nodes'] = ttk.Label(metrics_frame, text="0", font=("Arial", 10, "bold"))
        self.metrics_labels['nodes'].grid(row=0, column=1, padx=5)
        
        ttk.Label(metrics_frame, text="Path Cost:").grid(row=0, column=2, padx=5)
        self.metrics_labels['cost'] = ttk.Label(metrics_frame, text="0", font=("Arial", 10, "bold"))
        self.metrics_labels['cost'].grid(row=0, column=3, padx=5)
        
        ttk.Label(metrics_frame, text="Time (ms):").grid(row=0, column=4, padx=5)
        self.metrics_labels['time'] = ttk.Label(metrics_frame, text="0", font=("Arial", 10, "bold"))
        self.metrics_labels['time'].grid(row=0, column=5, padx=5)
        
        self.status_label = ttk.Label(self.root, text="", foreground="red")
        self.status_label.pack()
        
        instr_frame = ttk.Frame(self.root)
        instr_frame.pack(fill="x", padx=5, pady=2)
        
        instructions = "Click: Toggle obstacle | Shift+Click: Set Start (S) | Ctrl+Click: Set Goal (G)"
        ttk.Label(instr_frame, text=instructions, foreground="blue").pack()
    
    def update_settings_display(self):
        """Update the settings display label"""
        text = f"Current: {self.algorithm.get()} with {self.heuristic.get()} Heuristic"
        if hasattr(self, 'settings_label'):
            self.settings_label.config(text=text)
    
    def on_setting_change(self, event=None):
        """Called when algorithm or heuristic is changed"""
        self.update_settings_display()
        self.clear_all()
        print(f"Switched to: {self.algorithm.get()} with {self.heuristic.get()} heuristic")
    
    def draw_grid(self):
        self.canvas.delete("all")
        
        for row in range(self.rows):
            for col in range(self.cols):
                x1 = col * self.cell_size
                y1 = row * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                
                if self.grid.grid[row][col] == 1:
                    color = self.COLORS['obstacle']
                elif (row, col) == self.grid.start:
                    color = self.COLORS['start']
                elif (row, col) == self.grid.goal:
                    color = self.COLORS['goal']
                elif (row, col) in self.current_path:
                    color = self.COLORS['path']
                elif (row, col) in self.frontier_nodes:
                    color = self.COLORS['frontier']
                elif (row, col) in self.visited_nodes:
                    color = self.COLORS['visited']
                else:
                    color = self.COLORS['empty']
                
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='gray')
        
        if self.grid.start:
            x = self.grid.start[1] * self.cell_size + self.cell_size//2
            y = self.grid.start[0] * self.cell_size + self.cell_size//2
            self.canvas.create_text(x, y, text="S", font=("Arial", 10, "bold"))
        
        if self.grid.goal:
            x = self.grid.goal[1] * self.cell_size + self.cell_size//2
            y = self.grid.goal[0] * self.cell_size + self.cell_size//2
            self.canvas.create_text(x, y, text="G", font=("Arial", 10, "bold"))
        
        if self.agent_position and self.agent_position != self.grid.start:
            x = self.agent_position[1] * self.cell_size + self.cell_size//2
            y = self.agent_position[0] * self.cell_size + self.cell_size//2
            self.canvas.create_text(x, y, text="A", font=("Arial", 10, "bold"), fill="blue")
    
    def on_click(self, event):
        col = event.x // self.cell_size
        row = event.y // self.cell_size
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.grid.toggle_cell(row, col)
            self.clear_all()
            self.draw_grid()
    
    def on_shift_click(self, event):
        col = event.x // self.cell_size
        row = event.y // self.cell_size
        if 0 <= row < self.rows and 0 <= col < self.cols:
            if self.grid.goal != (row, col):
                self.grid.start = (row, col)
                self.grid.grid[row][col] = 0
                self.clear_all()
                self.draw_grid()
    
    def on_ctrl_click(self, event):
        col = event.x // self.cell_size
        row = event.y // self.cell_size
        if 0 <= row < self.rows and 0 <= col < self.cols:
            if self.grid.start != (row, col):
                self.grid.goal = (row, col)
                self.grid.grid[row][col] = 0
                self.clear_all()
                self.draw_grid()
    
    def find_path(self):
        if not self.grid.start or not self.grid.goal:
            messagebox.showwarning("Warning", "Please set both start and goal positions!")
            return
        
        self.visited_nodes.clear()
        self.frontier_nodes.clear()
        self.current_path = []
        self.status_label.config(text="")
        
        heuristic_func = get_heuristic(self.heuristic.get())
        
        if self.algorithm.get() == "A*":
            print(f"\nRunning A* with {self.heuristic.get()} heuristic...")
            result = astar_search(self.grid, self.grid.start, self.grid.goal, heuristic_func)
            path, visited, nodes_count, path_cost, time_taken, frontier = result
        else:
            print(f"\nRunning Greedy BFS with {self.heuristic.get()} heuristic...")
            result = greedy_bfs_search(self.grid, self.grid.start, self.grid.goal, heuristic_func)
            path, visited, nodes_count, path_cost, time_taken, frontier = result
        
        if path:
            self.current_path = path
            self.agent_position = self.grid.start
            self.visited_nodes = visited
            self.frontier_nodes = frontier
            
            self.metrics_labels['nodes'].config(text=str(nodes_count))
            self.metrics_labels['cost'].config(text=str(path_cost))
            self.metrics_labels['time'].config(text=f"{time_taken:.2f}")
            
            print(f"Path found! Length: {path_cost}, Nodes visited: {nodes_count}, Time: {time_taken:.2f}ms")
        else:
            print(" No path found to goal!")
            self.visited_nodes = visited
            self.frontier_nodes = frontier
            messagebox.showwarning("No Path", "No path found to goal!")
        
        self.draw_grid()
    
    def random_map(self):
        self.grid.generate_random_map(0.3)
        self.clear_all()
        self.draw_grid()
        print("New random map generated")
    
    def clear_all(self):
        self.current_path = []
        self.visited_nodes.clear()
        self.frontier_nodes.clear()
        self.agent_position = None
        self.status_label.config(text="")
        self.metrics_labels['nodes'].config(text="0")
        self.metrics_labels['cost'].config(text="0")
        self.metrics_labels['time'].config(text="0")
        self.draw_grid()
    
    def update_dynamic(self):
        if self.dynamic_mode.get() and self.agent_position and self.current_path:
            path_blocked = self.dynamic_manager.update(self.current_path, self.agent_position)
            
            self.status_label.config(text=" Dynamic Mode: New obstacles appearing!", foreground="red")
            
            if path_blocked:
                print("PATH BLOCKED! Recalculating...")
                self.status_label.config(text=" Path blocked! Finding new path...", foreground="red")
                self.root.update()  
                time.sleep(0.5)  
                self.find_path()
            else:
                if len(self.current_path) > 1:
                    self.agent_position = self.current_path[1]
                    self.current_path = self.current_path[1:]
                    self.draw_grid()
        else:
            self.status_label.config(text="")
        
        self.root.after(400, self.update_dynamic)
    
    def run(self):
        print("Starting main loop...")
        self.root.mainloop()



# ============================================================
# MAIN 
# ============================================================
if __name__ == "__main__":
    print("=" * 60)
    print("DYNAMIC PATHFINDING AGENT")
    print("=" * 60)
    print("\nIMPLEMENTED ALGORITHMS:")
    print("  • A* Search: f(n) = g(n) + h(n)")
    print("  • Greedy Best-First Search: f(n) = h(n)")
    print("\nHEURISTIC FUNCTIONS:")
    print("  • Manhattan Distance: |x1-x2| + |y1-y2|")
    print("  • Euclidean Distance: √[(x1-x2)² + (y1-y2)²]")
    print("\nCONTROLS:")
    print("  • Click: Toggle obstacle")
    print("  • Shift+Click: Set start position (S)")
    print("  • Ctrl+Click: Set goal position (G)")
    print("  • Dropdowns: Switch algorithms and heuristics")
    print("  • 'Find Path': Run selected algorithm")
    print("  • 'Random Map': Generate new random map")
    print("  • 'Clear': Clear current path")
    print("  • 'Dynamic Mode': Watch obstacles appear and path recalculate!")
    print("\n" + "=" * 60)
    
    try:
        app = PathfindingGUI(rows=15, cols=20)
        app.run()
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        input("\nPress Enter to exit...")
