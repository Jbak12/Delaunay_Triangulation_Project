import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Function to plot triangles given a list of vertices for each triangle
def plot_triangles(vertices_list):
    # Create a plot
    fig, ax = plt.subplots()
    
    # Initialize lists to store all x and y coordinates
    all_x = []
    all_y = []
    
    # Plot each triangle
    for vertices in vertices_list:
        triangle = patches.Polygon(vertices, closed=True, fill=None, edgecolor='r')
        ax.add_patch(triangle)
        
        # Add the x and y coordinates to the lists
        for x, y in vertices:
            all_x.append(x)
            all_y.append(y)
    
    # Set the plot limits to include all points
    ax.set_xlim(min(all_x) - 1, 10)
    ax.set_ylim(min(all_y) - 1, 10)
    
    # Add a grid for better visibility
    ax.grid(True)
    
    # Show the plot
    plt.show()

# Function to read triangles from a file
def read_triangles_from_file(filename):
    vertices_list = []
    with open(filename, 'r') as file:
        for line in file:
            # Split the line into coordinates and convert them to integers
            numbers = list(map(int, line.split()))
            if len(numbers) != 6:
                print(f"Invalid line: {line.strip()}. Each line must have exactly 6 numbers.")
                continue
            # Group the numbers into vertices
            vertices = [(numbers[i], numbers[i + 1]) for i in range(0, len(numbers), 2)]
            vertices_list.append(vertices)
    return vertices_list

# Example usage with the 'triangles.txt' file
vertices_list = read_triangles_from_file('triangles.txt')
plot_triangles(vertices_list)
