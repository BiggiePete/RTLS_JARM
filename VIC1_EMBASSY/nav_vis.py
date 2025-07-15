import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button
from matplotlib.animation import FuncAnimation
import re
import sys


def parse_calibration_file(filename):
    """Parse the calibration file and extract all position/rotation data"""
    try:
        # Try different encodings
        encodings = ['utf-16', 'utf-16-le', 'utf-16-be', 'utf-8', 'ascii']
        content = None
        used_encoding = None
        
        for encoding in encodings:
            try:
                with open(filename, 'r', encoding=encoding) as file:
                    content = file.read()
                    used_encoding = encoding
                    print(f"Successfully read file with encoding: {encoding}")
                    break
            except (UnicodeDecodeError, UnicodeError):
                continue
        
        if content is None:
            print("Could not read file with any supported encoding")
            return None
        
        # Try different variations of "begin"
        begin_markers = ["begin!!", "begin!", "begin", "Begin!!", "Begin!", "Begin"]
        begin_index = -1
        found_marker = None
        
        for marker in begin_markers:
            begin_index = content.find(marker)
            if begin_index != -1:
                found_marker = marker
                print(f"Found marker: '{marker}' at position {begin_index}")
                break
        
        if begin_index == -1:
            print("No 'begin' marker found, using all bracketed data in file")
            search_content = content
        else:
            search_content = content[begin_index + len(found_marker):]
        
        # Find all bracket pairs using regex
        bracket_pattern = r'\[([^\]]+)\]'
        matches = re.findall(bracket_pattern, search_content)
        
        if not matches:
            print("No bracketed data found")
            return None
        
        print(f"Found {len(matches)} data arrays")
        
        # Parse all bracket contents
        all_data = []
        for i, bracket_content in enumerate(matches):
            numbers = []
            for num_str in bracket_content.split():
                try:
                    # Clean the string of any null bytes or extra whitespace
                    cleaned = num_str.replace('\x00', '').strip()
                    if not cleaned:
                        continue
                    num = float(cleaned)
                    numbers.append(num)
                except ValueError:
                    continue
            
            if len(numbers) >= 6:
                position = np.array(numbers[:3])
                rotation_deg = np.array(numbers[3:6])  # In degrees
                rotation_rad = np.deg2rad(rotation_deg)  # Convert to radians
                all_data.append((position, rotation_rad, rotation_deg))
            
            # Show progress for large files
            if i % 1000 == 0 and i > 0:
                print(f"Processed {i} data points...")
        
        print(f"Successfully parsed {len(all_data)} complete data points")
        return all_data
            
    except FileNotFoundError:
        print(f"File '{filename}' not found")
        return None
    except Exception as e:
        print(f"Error parsing file: {e}")
        return None

def create_rocket_vertices():
    """Create a simple rocket shape"""
    # Rocket body (cylinder)
    height = 2.0
    radius = 0.3
    
    # Body vertices
    body_vertices = []
    # Bottom circle
    for i in range(8):
        angle = 2 * np.pi * i / 8
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        body_vertices.append([x, y, 0])
    
    # Top circle
    for i in range(8):
        angle = 2 * np.pi * i / 8
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        body_vertices.append([x, y, height])
    
    # Nose cone vertices
    nose_vertices = []
    for i in range(8):
        angle = 2 * np.pi * i / 8
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        nose_vertices.append([x, y, height])
    
    # Nose tip
    nose_vertices.append([0, 0, height + 0.8])
    
    # Fins
    fin_vertices = [
        # Fin 1
        [radius, 0, 0], [radius + 0.5, 0, 0], [radius + 0.3, 0, 0.6],
        # Fin 2
        [-radius, 0, 0], [-radius - 0.5, 0, 0], [-radius - 0.3, 0, 0.6],
        # Fin 3
        [0, radius, 0], [0, radius + 0.5, 0], [0, radius + 0.3, 0.6],
        # Fin 4
        [0, -radius, 0], [0, -radius - 0.5, 0], [0, -radius - 0.3, 0.6]
    ]
    
    return np.array(body_vertices), np.array(nose_vertices), np.array(fin_vertices)

def euler_to_rotation_matrix(roll, pitch, yaw):
    """Convert Euler angles (in radians) to rotation matrix"""
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    
    # Combined rotation matrix
    R = R_z @ R_y @ R_x
    return R

class RocketAnimator:
    def __init__(self, data):
        self.data = data
        self.current_frame = 0
        self.playing = False
        self.speed = 1.0
        self.animation = None
        
        # Create rocket geometry
        self.body_verts, self.nose_verts, self.fin_verts = create_rocket_vertices()
        
        # Set up the plot
        self.fig = plt.figure(figsize=(14, 10))
        
        # Main 3D plot
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.subplots_adjust(bottom=0.25)
        
        # Initialize plot elements
        self.trail_positions = []
        self.show_trail = False
        
        # Create control widgets
        self.setup_controls()
        
        # Initial plot
        self.update_plot()
        
        # Set up animation
        self.setup_animation()

    def setup_animation(self):
      """Set up the matplotlib animation"""
      self.animation = FuncAnimation(
          self.fig, self.animate_frame, frames=len(self.data),
          interval=50, repeat=False, blit=False
      )
      self.animation.pause()  # Start paused
    
    def animate_frame(self, frame):
      """Animation frame update function"""
      if self.playing:
          # Update frame based on speed
          self.current_frame = min(int(frame * self.speed), len(self.data) - 1)
          
          # Update slider without triggering callback
          self.frame_slider.eventson = False
          self.frame_slider.set_val(self.current_frame)
          self.frame_slider.eventson = True
          
          self.update_trail()
          self.update_plot()
          
      return []


    def setup_controls(self):
        """Set up the animation control widgets"""
        # Slider for frame selection
        ax_frame = plt.axes([0.1, 0.15, 0.6, 0.03])
        self.frame_slider = Slider(ax_frame, 'Frame', 0, len(self.data)-1, 
                                  valinit=0, valfmt='%d')
        self.frame_slider.on_changed(self.on_frame_change)
        
        # Play/Pause button
        ax_play = plt.axes([0.75, 0.15, 0.08, 0.04])
        self.play_button = Button(ax_play, 'Play')
        self.play_button.on_clicked(self.toggle_play)
        
        # Speed slider
        ax_speed = plt.axes([0.1, 0.1, 0.3, 0.03])
        self.speed_slider = Slider(ax_speed, 'Speed', 0.1, 5.0, 
                                  valinit=1.0, valfmt='%.1fx')
        self.speed_slider.on_changed(self.on_speed_change)
        
        # Reset button
        ax_reset = plt.axes([0.85, 0.15, 0.08, 0.04])
        self.reset_button = Button(ax_reset, 'Reset')
        self.reset_button.on_clicked(self.reset_animation)
        
        # Trail toggle button
        ax_trail = plt.axes([0.75, 0.1, 0.08, 0.04])
        self.trail_button = Button(ax_trail, 'Trail')
        self.trail_button.on_clicked(self.toggle_trail)
        self.show_trail = False
    
    def transform_vertices(self, vertices, position, rotation_rad):
        """Apply rotation and translation to vertices"""
        R = euler_to_rotation_matrix(rotation_rad[0], rotation_rad[1], rotation_rad[2])
        rotated = (R @ vertices.T).T
        translated = rotated + position
        return translated
    
    def update_plot(self):
        """Update the 3D plot with current frame data"""
        if not self.data:
            return
            
        position, rotation_rad, rotation_deg = self.data[self.current_frame]
        
        # Transform rocket vertices
        body_transformed = self.transform_vertices(self.body_verts, position, rotation_rad)
        nose_transformed = self.transform_vertices(self.nose_verts, position, rotation_rad)
        fin_transformed = self.transform_vertices(self.fin_verts, position, rotation_rad)
        
        # Clear previous plots
        self.ax.clear()
        
        # Plot rocket components
        self.ax.scatter(body_transformed[:, 0], body_transformed[:, 1], body_transformed[:, 2], 
                       c='blue', s=20, alpha=0.7, label='Body')
        self.ax.scatter(nose_transformed[:, 0], nose_transformed[:, 1], nose_transformed[:, 2], 
                       c='red', s=30, alpha=0.8, label='Nose')
        self.ax.scatter(fin_transformed[:, 0], fin_transformed[:, 1], fin_transformed[:, 2], 
                       c='green', s=15, alpha=0.6, label='Fins')
        
        # Draw rocket structure lines
        # Body cylinder lines
        for i in range(8):
            self.ax.plot([body_transformed[i, 0], body_transformed[i+8, 0]],
                        [body_transformed[i, 1], body_transformed[i+8, 1]],
                        [body_transformed[i, 2], body_transformed[i+8, 2]], 'b-', alpha=0.5)
        
        # Connect nose to body
        for i in range(8):
            self.ax.plot([nose_transformed[i, 0], nose_transformed[8, 0]],
                        [nose_transformed[i, 1], nose_transformed[8, 1]],
                        [nose_transformed[i, 2], nose_transformed[8, 2]], 'r-', alpha=0.5)
        
        # Add trail if enabled
        if self.show_trail and len(self.trail_positions) > 1:
            trail_array = np.array(self.trail_positions)
            self.ax.plot(trail_array[:, 0], trail_array[:, 1], trail_array[:, 2], 
                        'orange', alpha=0.6, linewidth=2, label='Trail')
        
        # Set labels and title
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title(f'Rocket Animation - Frame {self.current_frame}/{len(self.data)-1}\n' +
                         f'Position: [{position[0]:.6f}, {position[1]:.6f}, {position[2]:.6f}]\n' +
                         f'Rotation (deg): [{rotation_deg[0]:.2f}, {rotation_deg[1]:.2f}, {rotation_deg[2]:.2f}]')
        
        # Set axis limits based on data range
        all_positions = np.array([d[0] for d in self.data])
        margin = 2.0
        center = position
        
        self.ax.set_xlim([center[0] - margin, center[0] + margin])
        self.ax.set_ylim([center[1] - margin, center[1] + margin])
        self.ax.set_zlim([center[2] - margin, center[2] + margin])
        
        self.ax.legend()
        plt.draw()
    
    def on_frame_change(self, val):
        """Handle frame slider change"""
        # Pause animation when manually changing frame
        if self.playing:
            self.playing = False
            self.animation.pause()
            self.play_button.label.set_text('Play')
        
        self.current_frame = int(self.frame_slider.val)
        self.update_trail()
        self.update_plot()
    
    def on_speed_change(self, val):
        """Handle speed slider change"""
        self.speed = self.speed_slider.val
    
    def toggle_play(self, event):
        """Toggle play/pause"""
        self.playing = not self.playing
        self.play_button.label.set_text('Pause' if self.playing else 'Play')
        
        if self.playing:
            # Resume animation
            self.animation.resume()
        else:
            # Pause animation
            self.animation.pause()
    
    def reset_animation(self, event):
        """Reset animation to beginning"""
        self.current_frame = 0
        self.frame_slider.set_val(0)
        self.trail_positions = []
        self.update_plot()
    
    def toggle_trail(self, event):
        """Toggle trail visibility"""
        self.show_trail = not self.show_trail
        self.trail_button.label.set_text('Trail On' if self.show_trail else 'Trail')
        if self.show_trail:
            self.update_trail()
        self.update_plot()
    
    def update_trail(self):
        """Update trail positions up to current frame"""
        if self.show_trail:
            self.trail_positions = [self.data[i][0] for i in range(min(self.current_frame + 1, len(self.data)))]

def main():
    if len(sys.argv) != 2:
        print("Usage: python rocket_visualizer.py <filename>")
        return
    
    filename = sys.argv[1]
    
    # Parse the file
    data = parse_calibration_file(filename)
    if not data:
        print("Failed to parse data from file")
        return
    
    print(f"Loaded {len(data)} data points")
    print(f"Position range: X=[{min(d[0][0] for d in data):.6f}, {max(d[0][0] for d in data):.6f}]")
    print(f"               Y=[{min(d[0][1] for d in data):.6f}, {max(d[0][1] for d in data):.6f}]")
    print(f"               Z=[{min(d[0][2] for d in data):.6f}, {max(d[0][2] for d in data):.6f}]")
    
    # Create and run the animator
    animator = RocketAnimator(data)
    plt.show()

if __name__ == "__main__":
    main()