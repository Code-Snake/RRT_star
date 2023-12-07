import tkinter as tk
from tkinter import Canvas, Frame, Scrollbar, filedialog, Entry
from RRT_star import *
from Graph import Graph


class RRTStarGUI:
    def __init__(self, master):
        self.obstacles=[]
        self.master = master
        self.master.title("RRT-Star GUI")

        self.canvas = Canvas(master, width=800, height=600, bg="white")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.scrollbar = Scrollbar(master, command=self.canvas.yview)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.frame = Frame(self.canvas, bg="white")
        self.canvas.create_window((0, 0), window=self.frame, anchor="nw")

        self.init_scene()

        menu_bar = tk.Menu(master)
        master.config(menu=menu_bar)

        file_menu = tk.Menu(menu_bar, tearoff=0)
        menu_bar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Scene", command=self.save_scene)
        file_menu.add_command(label="Load Scene", command=self.load_scene)

        self.qinit_label = tk.Label(master, text="Qinit (x, y):")
        self.qinit_label.pack(pady=5)
        self.qinit_entry = Entry(master)
        self.qinit_entry.pack(pady=5)

        self.qgoal_label = tk.Label(master, text="Qgoal (x, y):")
        self.qgoal_label.pack(pady=5)
        self.qgoal_entry = Entry(master)
        self.qgoal_entry.pack(pady=5)

        self.triangle = tk.Label(master, text="Triangle:")
        self.triangle.pack(pady=5)
        self.A = tk.Label(master, text="A(x,y):")
        self.A.pack(pady=5)
        self.A_entry = Entry(master)
        self.A_entry.pack(pady=5)

        self.B = tk.Label(master, text="B(x,y):")
        self.B.pack(pady=5)
        self.B_entry = Entry(master)
        self.B_entry.pack(pady=5)

        self.C = tk.Label(master, text="C(x,y):")
        self.C.pack(pady=5)
        self.C_entry = Entry(master)
        self.C_entry.pack(pady=5)

        self.obstacles_button = tk.Button(master, text="Add triangle", command=self.add_obstacles)
        self.obstacles_button.pack(pady=10)

        self.N_label = tk.Label(master, text="N:")
        self.N_label.pack(pady=5)
        self.N_entry = Entry(master)
        self.N_entry.pack(pady=5)

        self.K_label = tk.Label(master, text="k:")
        self.K_label.pack(pady=5)
        self.K_entry = Entry(master)
        self.K_entry.pack(pady=5)

        self.start_button = tk.Button(master, text="Run RRT-Star", command=self.run_rrt_star)
        self.start_button.pack(pady=10)

        self.result = tk.Label(master, text="Result:")
        self.result.pack(pady=5)

        self.clear_button = tk.Button(master, text="Clear screen", command=self.clear_canvas)
        self.clear_button.pack(pady=10)

    def add_obstacles(self):
        A = [float(item) for item in self.A_entry.get().split(',')]
        B = [float(item) for item in self.B_entry.get().split(',')]
        C = [float(item) for item in self.C_entry.get().split(',')]
        self.obstacles.append(triangle(A[0],A[1],B[0],B[1],C[0],C[1]))
        print(self.obstacles)

    def init_scene(self):
        # Initialize your scene, obstacles, and other parameters here
        # Draw the initial scene on the canvas
        pass

    def save_scene(self):
        # Implement saving the current scene to a file
        file_path = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text files", "*.txt")])

        if file_path:
            with open(file_path, 'w') as file:
                # Write obstacles to the file
                for obstacle in self.obstacles:
                    file.write(f"{obstacle.x1},{obstacle.y1},{obstacle.x2},{obstacle.y2},{obstacle.x3},{obstacle.y3}\n")

                # Write Qinit, Qgoal, N, and k to the file
                file.write(f"Qinit: {self.qinit_entry.get()}\n")
                file.write(f"Qgoal: {self.qgoal_entry.get()}\n")
                file.write(f"N: {self.N_entry.get()}\n")
                file.write(f"k: {self.K_entry.get()}\n")

        print("Scene saved to:", file_path)

    def load_scene(self):
        # Implement loading a scene from a file
        file_path = filedialog.askopenfilename(filetypes=[("Text files", "*.txt")])

        if file_path:
            with open(file_path, 'r') as file:
                # Clear existing obstacles
                self.obstacles = []

                # Read obstacle coordinates from the file
                for line in file:
                    if line.strip():
                        coords = [float(coord) for coord in line.split(',') if
                                  coord.strip().replace('.', '', 1).isdigit()]
                        if len(coords) == 6:
                            obstacle = triangle(coords[0], coords[1], coords[2], coords[3], coords[4], coords[5])
                            self.obstacles.append(obstacle)

                # Update GUI with loaded obstacles
                self.draw_obstacles(self.obstacles)

                # Read and set Qinit, Qgoal, N, and k from the file
                file.seek(0)  # Reset file pointer to the beginning
                for line in file:
                    if line.startswith("Qinit"):
                        self.qinit_entry.delete(0, tk.END)
                        self.qinit_entry.insert(0, line.split(":")[1].strip())
                    elif line.startswith("Qgoal"):
                        self.qgoal_entry.delete(0, tk.END)
                        self.qgoal_entry.insert(0, line.split(":")[1].strip())
                    elif line.startswith("N"):
                        self.N_entry.delete(0, tk.END)
                        self.N_entry.insert(0, line.split(":")[1].strip())
                    elif line.startswith("k"):
                        self.K_entry.delete(0, tk.END)
                        self.K_entry.insert(0, line.split(":")[1].strip())
                qinit_str = self.qinit_entry.get()
                qgoal_str = self.qgoal_entry.get()
                qinit = tuple(map(float, qinit_str.split(',')))
                qgoal = tuple(map(float, qgoal_str.split(',')))
                self.draw_Q(qinit,qgoal)
        print("Scene loaded from:", file_path)

    def run_rrt_star(self):

        # Get Qinit and Qgoal from the entry fields
        qinit_str = self.qinit_entry.get()
        qgoal_str = self.qgoal_entry.get()
        N_str=self.N_entry.get()
        k_str=self.K_entry.get()

        # Convert Qinit and Qgoal strings to tuples
        qinit = tuple(map(float, qinit_str.split(',')))
        qgoal = tuple(map(float, qgoal_str.split(',')))
        N=int(N_str)
        k=int(k_str)
        flag=True
        self.draw_obstacles(self.obstacles)
        self.draw_Q(qinit, qgoal)
        for obs in self.obstacles:
            if (is_point_inside_triangle(qinit,obs.point_one,obs.point_two,obs.point_free)):
                self.result.config(text="Error: Qinit or Qgoal inside triangle")
                flag=False
                break
        if flag:
            graph=Graph()
            # Implement calling the RRT-Star algorithm here with Qinit and Qgoal
            path, graph = RRT_Star(N,k, qinit, qgoal, self.obstacles)
            
            if(path==[]):
                self.result.config(text="Path not found")
            else:
                total_weight = Cost(graph, path[-1])
                self.result.config(text=str(total_weight))
            # Display the graph, path, and obstacles
            self.draw_graph(graph)
            self.draw_path(path)


    def draw_Q(self,Qinit,Qgoal):
        self.canvas.create_oval(Qinit[0]-5,Qinit[1]-5,Qinit[0]+5,Qinit[1]+5, fill="red", width=2)
        self.canvas.create_oval(Qgoal[0] - 5, Qgoal[1] - 5, Qgoal[0] + 5, Qgoal[1] + 5, fill="red", width=2)

    def draw_graph(self, graph):
        # Implement drawing the graph on the canvas
        for edge in graph.get_edges():
            v1, v2 = edge
            self.canvas.create_line(v1[0], v1[1], v2[0], v2[1], fill="blue")

    def draw_path(self, path):
        # Implement drawing the path on the canvas
        for i in range(len(path) - 1):
            v1, v2 = path[i],path[i + 1]
            self.canvas.create_line(v1[0], v1[1], v2[0], v2[1], fill="green", width=8)

    def draw_obstacles(self, obstacles):
        # Implement drawing the obstacles on the canvas
        for obstacle in obstacles:
            self.canvas.create_polygon(
                obstacle.x1, obstacle.y1,
                obstacle.x2, obstacle.y2,
                obstacle.x3, obstacle.y3,
                fill="magenta", outline="black"
            )

    def clear_canvas(self):
        self.canvas.delete("all")
        self.obstacles=[]
        self.qinit_entry.delete(0, tk.END)
        self.qgoal_entry.delete(0, tk.END)
        self.N_entry.delete(0, tk.END)
        self.K_entry.delete(0, tk.END)


def main():
    root = tk.Tk()
    app = RRTStarGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
