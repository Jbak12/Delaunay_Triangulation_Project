import matplotlib.pyplot as plt
import matplotlib.patches as patches

def plot_triangles(vertices_list):
    fig, ax = plt.subplots()
    
    all_x = []
    all_y = []
    
    # Plot each triangle
    for vertices in vertices_list:
        triangle = patches.Polygon(vertices, closed=True, fill=None, edgecolor='r')
        ax.add_patch(triangle)
        
        
        for x, y in vertices:
            all_x.append(x)
            all_y.append(y)
    
    ax.set_xlim(min(all_x) - 1, 10)
    ax.set_ylim(min(all_y) - 1, 10)
    
    ax.grid(True)
    
    plt.show()

def read_triangles_from_file(filename):
    vertices_list = []
    with open(filename, 'r') as file:
        for line in file:
            numbers = list(map(int, line.split()))
            if len(numbers) != 6:
                print(f"Invalid line: {line.strip()}. Each line must have exactly 6 numbers.")
                continue
            vertices = [(numbers[i], numbers[i + 1]) for i in range(0, len(numbers), 2)]
            vertices_list.append(vertices)
    return vertices_list

vertices_list = read_triangles_from_file('triangles.txt')
plot_triangles(vertices_list)
