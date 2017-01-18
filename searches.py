import Queue
import heapq
import collections

class Vertex:
    def __init__(self, location):
        self.loc_id = location
        self.adjacent = collections.OrderedDict()
        self.heuristics = 0
        

    def add_neighbor(self, neighbor, weight=0, direction='D'):
        self.adjacent[neighbor] = weight
        

    def get_connections(self):
        l=[]
        for k,v in self.adjacent.items():
          l.append(k)
        return l
      
    def get_heuristics(self):
      return self.heuristics

    def get_id(self):
        return self.loc_id

    def get_time(self, neighbor):
        return self.adjacent[neighbor]

class Graph:
    def __init__(self):
        self.loc_dict = {}
        self.num_of_locations = 0

    def __iter__(self):
        return iter(self.loc_dict.values())

    def add_location(self, loc):
        self.num_of_locations = self.num_of_locations + 1
        new_location = Vertex(loc)
        self.loc_dict[loc] = new_location
        return new_location

    def get_location(self, n):
        if n in self.loc_dict:
            return self.loc_dict[n]
        else:
            return None

    def add_path(self, loc1, loc2, cost = 0):
        if loc1 not in self.loc_dict:
            self.add_location(loc1)
        if loc2 not in self.loc_dict:
            self.add_location(loc2)

        self.loc_dict[loc1].add_neighbor(self.loc_dict[loc2], cost)
  

    def get_locations(self):
        return self.loc_dict.keys()

def bfs(graph, start, goal):
    queue = []
    queue.append([start])
    while queue:
        path = queue.pop(0)
        loc = path[-1]
        if loc == goal:
            return path
        for locations in loc.get_connections():
            new_path = list(path)
            new_path.append(locations)
            queue.append(new_path)


            
def dfs(graph, start, goal):
    s = [(start,[start])]
    visited = []
    while s:
        (vertex,path) = s.pop()
        print vertex.get_id()
        
        if vertex not in visited:
          visited.append(vertex)
           
          l = [x for x in  vertex.get_connections() if x not in visited]  
          for ele in l[::-1]:
            s.append((ele,path+[ele]))
            if ele == goal:
              path= path +[ele]
              return path
            
def ucs(graph,start,goal):
    queue = [(0,start,[])]
    visited = {}
    #queue.append([start])
    while queue:
        
        heapq.heapify(queue)
        cost,node,path = heapq.heappop(queue)
        if visited.has_key(node) and visited[node]<cost:
          continue
        path = path + [node]
        loc = path[-1]
        if loc == goal:
            return path
        for locations in loc.get_connections():
          #print locations.get_id() 
          child_cost = loc.get_time(locations)
          #print cost
          #child_cost = 1 if i == goal else 0
          if locations not in visited:
            heapq.heappush(queue,(cost+child_cost,locations,path))
        visited[node]=cost
            #new_path = list(path)
            #new_path.append(locations)
            #queue.append(new_path)
  

def astar(graph,start,goal):
  
  queue = [(0,start,[])]
  visited = {}
  
  while queue: 
      heapq.heapify(queue)
      cost,node,path = heapq.heappop(queue)
      if visited.has_key(node) and visited[node]<cost:
        continue
      path = path + [node]
      loc = path[-1]
      if loc == goal:
        return path
      for locations in loc.get_connections():
        child_cost = loc.get_time(locations) + int(locations.get_heuristics())
        if locations not in visited:
          heapq.heappush(queue,(cost+child_cost,locations,path))
      visited[node]=cost

def bfs_dfs_output(path):
  fdw = open('output.txt','w')
  path_cost =0
  for loc in path:
    fdw.write("{} {} {}".format(loc,path_cost,'\n')) 
    path_cost = path_cost+1
  fdw.close()

def ucs_astar_output(path):
  fdw = open('output.txt','w')
  path_cost = cost = 0
  for i in range(len(path)-1):
    if i==0:
      cost = path[i].get_time(path[i+1])
      fdw.write("{} {} {}".format(path[i].get_id(),0,'\n'))
      path_cost = path_cost + cost
    else:
      fdw.write("{} {} {}".format(path[i].get_id(),path_cost,'\n'))
      cost = path[i].get_time(path[i+1])
      path_cost = path_cost + cost
  fdw.write("{} {} {}".format(path[-1].get_id(),path_cost,'\n'))
  fdw.close()
      
if __name__ == '__main__':
  fd = open('input.txt','r')
  #TEST
  #fdw = open('output.txt','w')
  input_list = fd.readlines()
  algo = input_list[0].strip('\n')
  start = input_list[1].strip('\n')
  goal = input_list[2].strip('\n')
  lt_lines= input_list[3]
  #print "Algo:{}".format(algo)
  #print "Start:{}".format(start)
  #print "Goal:{}".format(goal)
  #print int(lt_lines)
  if start==goal:
    fdw = open('output.txt','w')
    fdw.write('{} 0 {}'.format(start,'\n'))
    fdw.close()
    exit(0)
              
              
  g = Graph()
  for i in range(0,int(lt_lines)):
    lt = input_list[4+i].split(" ")
    g.add_path(lt[0],lt[1],int(lt[2]))
    #print g
    #print lt
  st_lines = input_list[4+int(lt_lines)]
  
  for j in range(0,int(st_lines)):
    #print input_list[5+int(lt_lines)+j]
    st = input_list[5+int(lt_lines)+j].split(" ")
    loc_id = st[0]
    g.get_location(st[0]).heuristics = st[1]
  
  ##Test
  for v in g:
    for w in v.get_connections():
      vid = v.get_id()
      wid = w.get_id()
      print '( %s , %s, %3d)'%( vid, wid, v.get_time(w))
      

  #for v in g:
    #print 'g.vert_dict[%s]=%s' %(v.get_id(), g.loc_dict[v.get_id()])
  
  for v in g:
    if v.get_id() == start:
      start_vertex = v
    if v.get_id() == goal:
      goal_vertex = v
  if algo == "BFS":
    output = []
    explored= bfs(g,start_vertex,goal_vertex)
    #print explored
    for x in explored:
        output.append(x.get_id())
    #print output
    bfs_dfs_output(output)
  elif algo == "DFS":
    output = []
    explored= dfs(g,start_vertex,goal_vertex)
    for x in explored:
        output.append(x.get_id())
    bfs_dfs_output(output)
  elif algo == "UCS":
    #explored= ucs(g,start,goal)
    explored= ucs(g,start_vertex,goal_vertex)
    ucs_astar_output(explored)
  elif algo == "A*":
    explored= astar(g,start_vertex,goal_vertex)
    ucs_astar_output(explored)
  else:
    print "ERROR"
    
  
    
    
