import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation

# --- Constants ---
CUBE_SIDE_LENGTH = 1.0

def parse_log_file(filepath):
    """
    Reads an entire log file, handling UTF-16 encoding and ignoring non-data lines.
    Returns a NumPy array of all valid state vectors found.
    """
    states = []
    print(f"Reading and parsing log file: {filepath}")
    
    try:
        # --- THIS IS THE KEY FIX for the "weird decoding" ---
        # PowerShell's '>' redirection creates a UTF-16 encoded file on Windows.
        # We must open it with the correct encoding to read it properly.
        with open(filepath, 'r', encoding='utf-16', errors='ignore') as f:
            for line in f:
                # This try/except block robustly ignores all startup "garbage"
                # and any other malformed lines.
                try:
                    if '[' in line and ']' in line:
                        list_str = line.split('[')[1].split(']')[0]
                        state_vector = [float(x) for x in list_str.split(',')]
                        # Ensure it's a valid [pos, quat] vector
                        if len(state_vector) == 7:
                            states.append(state_vector)
                except (ValueError, IndexError):
                    continue # Ignore this line and move to the next
    except FileNotFoundError:
        print(f"Error: File not found at '{filepath}'")
        sys.exit(1)
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        sys.exit(1)

    print(f"Found {len(states)} valid state vectors.")
    return np.array(states)

def animate_from_data(states):
    """
    Creates and displays a 3D animation from a pre-loaded array of state data.
    """
    if states.size == 0:
        print("No data to animate.")
        return

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # --- SETUP THE SCENE (DONE ONCE) ---

    # 1. Extract all positions to determine plot boundaries and draw trajectory
    positions = states[:, 0:3]
    
    # Draw the entire trajectory at once
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'k:', alpha=0.5, label='Trajectory')

    # Calculate the boundaries for the entire flight path
    min_coords = positions.min(axis=0)
    max_coords = positions.max(axis=0)
    # Create a buffer around the trajectory
    center = (max_coords + min_coords) / 2
    max_range = (max_coords - min_coords).max() * 0.6  # 60% of max range for buffer
    
    ax.set_xlim(center[0] - max_range, center[0] + max_range)
    ax.set_ylim(center[1] - max_range, center[1] + max_range)
    ax.set_zlim(center[2] - max_range, center[2] + max_range)

    # 2. Define the cube model
    s = CUBE_SIDE_LENGTH / 2
    vertices = np.array([
        [-s,-s,-s],[s,-s,-s],[s,s,-s],[-s,s,-s],
        [-s,-s,-s],[s,-s,s],[s,s,s],[-s,s,s]
    ])
    faces = [[vertices[j] for j in [0,1,2,3]],[vertices[j] for j in [4,5,6,7]],
             [vertices[j] for j in [0,1,5,4]],[vertices[j] for j in [2,3,7,6]],
             [vertices[j] for j in [1,2,6,5]],[vertices[j] for j in [3,0,4,7]]]
    
    # NOTE: Correcting a typo from the old script (cube__poly -> cube_poly)
    cube_poly = Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='b', alpha=.75)
    ax.add_collection3d(cube_poly)

    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title('Log File Visualization'); ax.legend()
    ax.set_box_aspect([1, 1, 1])

    # --- ANIMATION FUNCTION ---
    def update(frame):
        # Get the state for the current frame from the pre-loaded array
        current_state = states[frame]
        pos = current_state[0:3]
        qw, qx, qy, qz = current_state[3:7]
        
        # Perform rotation and translation
        quat_xyzw = [qx, qy, qz, qw]
        rotation = Rotation.from_quat(quat_xyzw)
        rotated_vertices = rotation.apply(vertices)
        world_vertices = rotated_vertices + pos
        
        # Update the cube's visual representation
        cube_poly.set_verts([
            [world_vertices[j] for j in [0,1,2,3]],[world_vertices[j] for j in [4,5,6,7]],
            [world_vertices[j] for j in [0,1,5,4]],[world_vertices[j] for j in [2,3,7,6]],
            [world_vertices[j] for j in [1,2,6,5]],[world_vertices[j] for j in [3,0,4,7]]
        ])
        
        return (cube_poly,)

    # Create and show the animation
    ani = FuncAnimation(fig, update, frames=len(states), 
                        interval=30, blit=False, repeat=True)
    plt.show()

# --- MAIN EXECUTION ---
if __name__ == "__main__":
    # Use 'live_log.txt' by default, or a path from the command line
    if len(sys.argv) > 1:
        log_filepath = sys.argv[1]
    else:
        log_filepath = 'live_log.txt'
    
    # Step 1: Parse the entire file
    all_states = parse_log_file(log_filepath)

    # Step 2: Animate the data we found
    animate_from_data(all_states)