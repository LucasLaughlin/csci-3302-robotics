import copy
from numpy import random
import numpy
import math 

g_CYCLE_TIME = .100


# Default parameters will create a 4x4 grid to test with
g_MAP_SIZE_X = 2. # 2m wide
g_MAP_SIZE_Y = 1.5 # 1.5m tall
g_MAP_RESOLUTION_X = 0.5 # Each col represents 50cm
g_MAP_RESOLUTION_Y = 0.375 # Each row represents 37.5cm
g_NUM_X_CELLS = int(g_MAP_SIZE_X // g_MAP_RESOLUTION_X) # Number of columns in the grid map 
g_NUM_Y_CELLS = int(g_MAP_SIZE_Y // g_MAP_RESOLUTION_Y) # Number of rows in the grid map

# Map from Lab 4: values of 0 indicate free space, 1 indicates occupied space
g_WORLD_MAP = [0] * g_NUM_Y_CELLS*g_NUM_X_CELLS # Initialize graph (grid) as array

# Source and Destination (I,J) grid coordinates
g_dest_coordinates = (3,3)
g_src_coordinates = (0,0)



def create_test_map(map_array):
  # Takes an array representing a map of the world, copies it, and adds simulated obstacles
  num_cells = len(map_array)
  map_matrix = copy.copy(map_array) 
  global g_dest_coordinates, g_src_coordinates, g_WORLD_MAP
  # Add obstacles to up to sqrt(n) vertices of the map
  for i in range(int(math.sqrt(len(map_array)))):
    random_cell = random.randint(0, num_cells)
    if random_cell != ij_to_vertex_index(g_src_coordinates[0],g_src_coordinates[1]):
        if random_cell != ij_to_vertex_index(g_dest_coordinates[0],g_dest_coordinates[1]):
            map_matrix[random_cell] = 1
  
  g_WORLD_MAP = map_matrix
  print(map_matrix)
  return map_matrix

def vertex_index_to_ij(vertex_index):
  '''
  vertex_index: unique ID of graph vertex to be convered into grid coordinates
  Returns COL, ROW coordinates in 2D grid
  '''
  global g_NUM_X_CELLS
  return vertex_index % g_NUM_X_CELLS, vertex_index // g_NUM_X_CELLS

def ij_to_vertex_index(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns integer 'vertex index'
  '''
  global g_NUM_X_CELLS
  return j*g_NUM_X_CELLS + i


def ij_coordinates_to_xy_coordinates(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return (i+0.5)*g_MAP_RESOLUTION_X, (j+0.5)*g_MAP_RESOLUTION_Y

def xy_coordinates_to_ij_coordinates(x,y):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return int(x // g_MAP_RESOLUTION_X), int(y // g_MAP_RESOLUTION_Y)

# **********************************
# *      Core Dijkstra Functions   *
# **********************************

def get_travel_cost(vertex_source, vertex_dest):
  # Returns the cost of moving from vertex_source (int) to vertex_dest (int)
  '''
      This function should return 1 if:
        vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)

      This function should return 1000 if:
        vertex_source corresponds to (i,j) coordinates outside the map
        vertex_dest corresponds to (i,j) coordinates outside the map
        vertex_source and vertex_dest are not adjacent to each other (i.e., more than 1 move away from each other)
  '''
  global g_NUM_Y_CELLS, g_NUM_X_CELLS, g_WORLD_MAP  # Number of columns in the grid map 

  (x_source,y_source) = vertex_index_to_ij(vertex_source)
  (x_dest,y_dest) = vertex_index_to_ij(vertex_dest)
  
  #vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)
  #print (g_WORLD_MAP)
  if g_WORLD_MAP[vertex_dest]==1 or g_WORLD_MAP[vertex_source]==1:
      return 1000
  if vertex_source == vertex_dest:
      return 0
  if vertex_source < len(g_WORLD_MAP) and vertex_dest < len(g_WORLD_MAP):
    start_i, start_j = vertex_index_to_ij(vertex_source)
    dest_i, dest_j = vertex_index_to_ij(vertex_dest)
    manDist = abs(start_i - dest_i) + abs(start_j - dest_j)
    if manDist == 1 and g_WORLD_MAP[vertex_source] != 1 and g_WORLD_MAP[vertex_dest] != 1:
      return 1

    return 100


def run_dijkstra(source_vertex):
  '''
  source_vertex: vertex index to find all paths back to
  returns: 'prev' array from a completed Dijkstra's algorithm run

  Function to return an array of ints corresponding to the 'prev' variable in Dijkstra's algorithm
  The 'prev' array stores the next vertex on the best path back to source_vertex.
  Thus, the returned array prev can be treated as a lookup table:  prev[vertex_index] = next vertex index on the path back to source_vertex
  '''  
  global g_NUM_X_CELLS, g_NUM_Y_CELLS
  
  # Array mapping vertex_index to distance of shortest path from vertex_index to source_vertex.
  dist = [999] * g_NUM_X_CELLS * g_NUM_Y_CELLS

  # Queue for identifying which vertices are up to still be explored:
  # Will contain tuples of (vertex_index, cost), sorted such that the min cost is first to be extracted (explore cheapest/most promising vertices first)
  Q_cost = [(x,get_travel_cost(source_vertex, x)) for x in range(g_NUM_X_CELLS*g_NUM_Y_CELLS)]
  

  # Array of ints for storing the next step (vertex_index) on the shortest path back to source_vertex for each vertex in the graph
  prev = [-1] * g_NUM_X_CELLS*g_NUM_Y_CELLS
  
  while (Q_cost):

    cur_vertex = min(Q_cost,key = lambda t: t[1]) #might need to possibly change to first value of a sorted Q_cost
    #print(cur_vertex)
    dist[cur_vertex[0]] = cur_vertex[1] 
    neighbors=[]
    (cur_x, cur_y) = vertex_index_to_ij(cur_vertex[0])
    neighbors.append(ij_to_vertex_index(cur_x, cur_y+1))
    neighbors.append(ij_to_vertex_index(cur_x+1, cur_y))
    neighbors.append(ij_to_vertex_index(cur_x-1, cur_y))
    neighbors.append(ij_to_vertex_index(cur_x, cur_y-1))
    Q_cost.remove(cur_vertex)
    #print(Q_cost)
    for i in neighbors:
      for q in Q_cost:
        if i == q[0]:
          #print(i)
          cost = get_travel_cost(cur_vertex[0], i) + cur_vertex[1]
          if cost< dist[i]:
            #print("hello")
            dist[i]= cost
            prev[i] = cur_vertex[0]

            Q_cost.remove(q)
            Q_cost.insert(0,(i,dist[i]))
   
  	

  # Return results of algorithm run
  
  return prev


def reconstruct_path(prev, source_vertex, dest_vertex):
  '''
  Given a populated 'prev' array, a source vertex_index, and destination vertex_index,
  allocate and return an integer array populated with the path from source to destination.
  The first entry of your path should be source_vertex and the last entry should be the dest_vertex.
  If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the
  path from dest to source, return an empty list.
  '''
  final_path = []
	
  vertex = dest_vertex
  final_path.append(dest_vertex)
  while vertex != source_vertex:
    if prev[vertex] == -1:
      print("no path available")
      return []
    final_path.insert(0,prev[vertex])
    vertex = prev[vertex]

  return (final_path)


def render_map(map_array): 
  global g_NUM_X_CELLS, g_NUM_Y_CELLS, g_WORLD_MAP
  
  out = [["." for x in range(g_NUM_X_CELLS)] for y in range(g_NUM_Y_CELLS)]
  for j in range(g_NUM_Y_CELLS):
        for i in range(g_NUM_X_CELLS):
            if (g_WORLD_MAP[ij_to_vertex_index(i,j)]== 0):
                out[i][j]=" . "
            elif (g_WORLD_MAP[ij_to_vertex_index(i,j)]== 1):
                out[i][j]="[ ]"
  out = numpy.transpose(out)
  print('\n'.join([''.join(['{:2}'.format(item) for item in (rowOut)]) for rowOut in reversed(out)]))
  print("\n")
  pass
  


def main():
  global g_WORLD_MAP, g_src_coordinates, g_dest_coordinates, Q_cost
  test_map = create_test_map(g_WORLD_MAP)
  g_WORLD_MAP = test_map

  # Use render_map to render your initialized obstacle map
  print("starting vertex: " ,g_src_coordinates)
  print("destination vertex: " ,g_dest_coordinates)

  render_map(test_map)
  prev = run_dijkstra(ij_to_vertex_index(g_src_coordinates[0],g_src_coordinates[1]))
  
  path = reconstruct_path(prev, ij_to_vertex_index(g_src_coordinates[0],g_src_coordinates[1]), ij_to_vertex_index(g_dest_coordinates[0],g_dest_coordinates[1]))
  
 
  for i in range(0, len(path)): 
    path[i] = str(path[i]) 
  
  print("path: "," -> ".join(path))
 
  


if __name__ == "__main__":
  main()

#iajipduvips

