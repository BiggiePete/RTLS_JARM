import pygame
import numpy as np
import re
import time
import os
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import threading
import queue

class QuaternionVisualizer:
    def __init__(self, filename="data.txt"):
        self.filename = filename
        self.quaternion = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z
        self.position = [0.0, 0.0, 0.0]  # x, y, z position
        self.data_queue = queue.Queue()
        self.running = True
        
        # Initialize Pygame and OpenGL
        pygame.init()
        self.display = (800, 600)
        pygame.display.set_mode(self.display, DOUBLEBUF | OPENGL)
        pygame.display.set_caption("Real-time Quaternion Visualization")
        
        # Set up OpenGL
        glEnable(GL_DEPTH_TEST)
        gluPerspective(45, (self.display[0] / self.display[1]), 0.1, 50.0)
        glTranslatef(0.0, 0.0, -5)
        
        # File monitoring
        self.file_pos = 0
        
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix"""
        w, x, y, z = q
        
        # Normalize quaternion
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        if norm > 0:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
        
        # Convert to rotation matrix
        matrix = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y), 0],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x), 0],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y), 0],
            [0, 0, 0, 1]
        ])
        
        return matrix.flatten()
    
    def draw_cube(self):
        """Draw a cube with different colored faces"""
        vertices = [
            [1, 1, -1], [1, -1, -1], [-1, -1, -1], [-1, 1, -1],  # Back face
            [1, 1, 1], [1, -1, 1], [-1, -1, 1], [-1, 1, 1]       # Front face
        ]
        
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # Back face
            (4, 5), (5, 6), (6, 7), (7, 4),  # Front face
            (0, 4), (1, 5), (2, 6), (3, 7)   # Connecting edges
        ]
        
        faces = [
            (0, 1, 2, 3),  # Back
            (4, 7, 6, 5),  # Front
            (0, 4, 5, 1),  # Right
            (2, 6, 7, 3),  # Left
            (0, 3, 7, 4),  # Top
            (1, 5, 6, 2)   # Bottom
        ]
        
        colors = [
            (1, 0, 0),    # Red - Back
            (0, 1, 0),    # Green - Front
            (0, 0, 1),    # Blue - Right
            (1, 1, 0),    # Yellow - Left
            (1, 0, 1),    # Magenta - Top
            (0, 1, 1)     # Cyan - Bottom
        ]
        
        # Draw faces
        glBegin(GL_QUADS)
        for i, face in enumerate(faces):
            glColor3fv(colors[i])
            for vertex in face:
                glVertex3fv(vertices[vertex])
        glEnd()
        
        # Draw edges for better visibility
        glColor3f(0, 0, 0)  # Black edges
        glBegin(GL_LINES)
        for edge in edges:
            for vertex in edge:
                glVertex3fv(vertices[vertex])
        glEnd()
    
    def parse_line(self, line):
        """Parse a line to extract position and quaternion data"""
        if "INFO" in line and "[" in line and "]" in line:
            # Extract the array from the line
            match = re.search(r'\[(.*?)\]', line)
            if match:
                try:
                    # Parse the comma-separated values
                    values = [float(x.strip()) for x in match.group(1).split(',')]
                    if len(values) >= 7:
                        # First 3 values are position (x, y, z)
                        # Next 4 values are quaternion (w, x, y, z)
                        position = values[:3]
                        quaternion = values[3:7]
                        return position, quaternion
                except ValueError:
                    pass
        return None
    
    def file_monitor(self):
        """Monitor file for new data"""
        while self.running:
            try:
                if os.path.exists(self.filename):
                    with open(self.filename, 'r', encoding='utf-8', errors='ignore') as f:
                        f.seek(self.file_pos)
                        lines = f.readlines()
                        self.file_pos = f.tell()
                        
                        for line in lines:
                            parsed_data = self.parse_line(line)
                            if parsed_data:
                                position, quaternion = parsed_data
                                self.data_queue.put((position, quaternion))
                
                time.sleep(0.01)  # Check every 10ms
            except Exception as e:
                print(f"Error reading file: {e}")
                time.sleep(0.1)
    
    def run(self):
        """Main visualization loop"""
        # Start file monitoring thread
        file_thread = threading.Thread(target=self.file_monitor, daemon=True)
        file_thread.start()
        
        clock = pygame.time.Clock()
        
        print(f"Starting visualization...")
        print(f"Monitoring file: {self.filename}")
        print("Controls:")
        print("- ESC or close window to quit")
        print("- Cube will rotate and translate based on position and quaternion data from file")
        
        while self.running:
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
            
            # Update position and quaternion from queue
            try:
                while not self.data_queue.empty():
                    self.position, self.quaternion = self.data_queue.get_nowait()
                    print(f"Position: {self.position}, Quaternion: {self.quaternion}")
            except queue.Empty:
                pass
            
            # Clear screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            
            # Apply transformations
            glPushMatrix()
            
            # Apply position translation (scaled for visibility)
            glTranslatef(self.position[0] * 10, self.position[1] * 10, self.position[2] * 10)
            
            # Convert quaternion to rotation matrix and apply
            rotation_matrix = self.quaternion_to_rotation_matrix(self.quaternion)
            glMultMatrixf(rotation_matrix)
            
            # Draw the cube
            self.draw_cube()
            
            glPopMatrix()
            
            # Update display
            pygame.display.flip()
            clock.tick(60)  # 60 FPS
        
        pygame.quit()

if __name__ == "__main__":
    import sys
    
    # Get filename from command line argument or use default
    filename = sys.argv[1] if len(sys.argv) > 1 else "data.txt"
    
    try:
        visualizer = QuaternionVisualizer(filename)
        visualizer.run()
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure you have pygame and PyOpenGL installed:")
        print("pip install pygame PyOpenGL PyOpenGL_accelerate numpy")