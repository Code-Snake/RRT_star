
from Graph import Graph
from random import randint
from math import sqrt, inf
import matplotlib.pyplot as plt

X_MAX = 800
Y_MAX= 600

class triangle:
    def __init__(self, x1, y1, x2, y2, x3, y3):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.x3 = x3
        self.y3 = y3
        self.point_one=(x1,y1)
        self.point_two = (x2, y2)
        self.point_free = (x3, y3)
        self.points=[self.point_one,self.point_two,self.point_free]

def is_point_inside_triangle(p, a, b, c):
    def sign(p1, p2, p3):
        return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

    d1 = sign(p, a, b)
    d2 = sign(p, b, c)
    d3 = sign(p, c, a)

    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)

    return not (has_neg and has_pos)

def is_segment_intersects_triangle(segment_start, segment_end, triangle):
    a, b, c = (triangle.x1, triangle.y1), (triangle.x2, triangle.y2), (triangle.x3, triangle.y3)

    # Check if any of the segment endpoints is inside the triangle
    if (
        is_point_inside_triangle(segment_start, a, b, c) or
        is_point_inside_triangle(segment_end, a, b, c)
    ):
        return True

    # Check if any of the triangle vertices is inside the segment
    if (
        (segment_start[0] <= a[0] <= segment_end[0] or segment_end[0] <= a[0] <= segment_start[0]) and
        (segment_start[1] <= a[1] <= segment_end[1] or segment_end[1] <= a[1] <= segment_start[1])
    ):
        return True

    if (
        (segment_start[0] <= b[0] <= segment_end[0] or segment_end[0] <= b[0] <= segment_start[0]) and
        (segment_start[1] <= b[1] <= segment_end[1] or segment_end[1] <= b[1] <= segment_start[1])
    ):
        return True

    if (
        (segment_start[0] <= c[0] <= segment_end[0] or segment_end[0] <= c[0] <= segment_start[0]) and
        (segment_start[1] <= c[1] <= segment_end[1] or segment_end[1] <= c[1] <= segment_start[1])
    ):
        return True

    # Check if the segment intersects any of the triangle edges, including the vertices
    if (
        is_segment_intersects(segment_start, segment_end, a, b) or
        is_segment_intersects(segment_start, segment_end, b, c) or
        is_segment_intersects(segment_start, segment_end, c, a)
    ):
        return True

    return False

def is_segment_intersects(p1, q1, p2, q2):
    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0
        return 1 if val > 0 else 2

    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    if o1 != o2 and o3 != o4:
        return True

    if o1 == 0 and on_segment(p1, p2, q1):
        return True

    if o2 == 0 and on_segment(p1, q2, q1):
        return True

    if o3 == 0 and on_segment(p2, p1, q2):
        return True

    if o4 == 0 and on_segment(p2, q1, q2):
        return True

    return False

def on_segment(p, q, r):
    return (
        (q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and
        (q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))
    )

def RandomSample():  # возвращает рандомные координаты точки
    x = randint(1, X_MAX)
    y = randint(1, Y_MAX)

    return (x,y)

def Length(X, Y):
    return sqrt((Y[0] - X[0]) ** 2 + (Y[1] - X[1]) ** 2)

def CollisionFree(X, Y, obstacles):
    # Проверка на коллизии между отрезком XY и препятствиями из Cobs
    for obstacle in obstacles:
        if (is_segment_intersects_triangle(X,Y,obstacle)):
            return False
    return True

def Nearest(G, X,obstacles):
    min_distance = float('inf')
    nearest_vertex = None
    nearest_edge = None

    for vertex in G.get_vertices():
        distance = Length(X, vertex)
        if distance < min_distance and distance!=0.0:
            min_distance = distance
            nearest_vertex = vertex
            nearest_edge = None

    for edge in G.edges:
        v1, v2 = edge
        distance = DistanceToLineSegment(X, v1, v2)
        if distance < min_distance and distance!=0.0:
            min_distance = distance
            nearest_vertex = None
            nearest_edge = edge

    if nearest_edge is not None:
        # Разделение ребра и добавление новой вершины
        new_vertex = Steer(G,nearest_edge[0], nearest_edge[1],obstacles)
        G.add_vertex(new_vertex)
        G.add_edge(nearest_edge[0], new_vertex)
        G.add_edge(new_vertex, nearest_edge[1])
        return new_vertex
    else:
        return nearest_vertex

def DistanceToLineSegment(P, A, B):
    # Расстояние от точки P до отрезка AB
    def dot(v, w):
        return v[0] * w[0] + v[1] * w[1]

    def length(v):
        return sqrt(v[0] ** 2 + v[1] ** 2)

    def vector(p1, p2):
        return [p2[0] - p1[0], p2[1] - p1[1]]

    v = vector(A, P)
    w = vector(A, B)
    c1 = dot(v, w)
    if c1 <= 0:
        return length(vector(A, P))
    c2 = dot(w, w)
    if c2 <= c1:
        return length(vector(B, P))
    b = c1 / c2
    Pb = [A[0] + b * w[0], A[1] + b * w[1]]
    return length(vector(P, Pb))

def Steer(G,X, Y, obstacles ):
    scale_factor = randint(5,10)
    distance_xy = Length(X, Y)

    # Check if the length is zero
    if distance_xy == 0:
        return X

    dx = (abs(Y[0] - X[0]) / distance_xy) * scale_factor
    dy = (abs(Y[1] - X[1]) / distance_xy) * scale_factor
    Z = list(Y)  # Convert Y to a list so it can be modified
    while (
            not CollisionFree(X, Z, obstacles)
            and 0 < Z[0] < X_MAX
            and 0 < Z[1] < Y_MAX
            and not Z in G.get_vertices()
    ):
        Z[0] -= dx
        Z[1] -= dy
    return tuple(Z)  # Convert Z back to a tuple before returning

def Near(G, X, k):
    distances = [(vertex, Length(X, vertex)) for vertex in G.get_vertices()]
    distances.sort(key=lambda x: x[1])
    nearest_vertices = [vertex for vertex, _ in distances[:k] if vertex != X]
    return nearest_vertices

def ChooseParent(G, Qs, Qn, Qnear, obstacles):
    Qmin = Qn
    Cmin = Cost(G, Qn) + Length(Qn, Qs)

    for Q in Qnear:
        if CollisionFree(Q, Qs, obstacles):
            current_cost = Cost(G, Q) + Length(Q, Qs)
            if current_cost < Cmin:
                Qmin = Q
                Cmin = current_cost

    return Qmin

def Update(G, Qs, Qnear, Qinit, obstacles):
    for Q in Qnear:
        if Qs != Q:  # Check if Qs and Q are different
            if CollisionFree(Qs, Q, obstacles) and Cost(G, Qs) + Length(Qs, Q) < Cost(G, Q) and Qinit != Parent(G, Q):
                # Remove the old edge
                G.remove_edge(Parent(G, Q), Q)
                # Add the new edge
                G.add_edge(Qs, Q)

def Parent(G, V):
    for edge in G.edges:
        v1, v2 = edge
        if V == v2 and V != v1:
            return v1
    return None

def Cost(G, V):
    total_cost = 0
    current_vertex = V

    while True:
        parent = Parent(G, current_vertex)

        if parent is None:
            break
        total_cost += Length(parent, current_vertex)
        current_vertex = parent

    return total_cost

def shortest_path(graph, start_vertex, end_vertex):
    if len(graph.get_vertices()) < 2 or len(graph.get_adjacent(end_vertex)) < 1 or start_vertex == end_vertex:
        return []

    result = []
    distance = {}
    vertices = graph.get_vertices()

    for i in range(len(vertices)):
        distance[vertices[i]] = float('inf')

    parent = {}
    for i in range(len(vertices)):
        parent[vertices[i]] = -1

    distance[start_vertex] = 0

    while vertices:
        min_distance = distance[vertices[0]]
        min_distance_vertex = vertices[0]

        for i in range(len(vertices)):
            if distance[vertices[i]] < min_distance:
                min_distance = distance[vertices[i]]
                min_distance_vertex = vertices[i]

        vertices.remove(min_distance_vertex)

        if min_distance_vertex == end_vertex:
            vertex = end_vertex

            while vertex != start_vertex:
                result.insert(0, vertex)
                vertex = parent[vertex]
                if vertex == -1:
                    return []
            result.insert(0, start_vertex)

            return result

        adjacent_vertices = graph.get_adjacent(min_distance_vertex)
        for i in range(len(adjacent_vertices)):
            if distance[adjacent_vertices[i]] > distance[min_distance_vertex] + graph.get_edge_weight(min_distance_vertex, adjacent_vertices[i]):
                distance[adjacent_vertices[i]] = distance[min_distance_vertex] + graph.get_edge_weight(min_distance_vertex, adjacent_vertices[i])
                parent[adjacent_vertices[i]] = min_distance_vertex


def RRT_Star(N,k, Qinit, Qgoal,obstacles):
    G = Graph()
    G.add_vertex(Qinit)

    for i in range(1,N):

        if i%10 ==0:
            print("Iteration: ",i)
        Qrand = RandomSample()
        while Qrand in G.get_vertices():
            Qrand = RandomSample()
        Qn = Nearest(G, Qrand,obstacles)

        Qs = Steer(G,Qn, Qrand,obstacles)
        if Qs[0]<=0 or Qs[1]<=0 or Qs[0]>=X_MAX or Qs[1]>=Y_MAX:
            while Qs[0]<0 or Qs[1]<0 or Qs[0]>=X_MAX or Qs[1]>=Y_MAX:
                Qrand = RandomSample()
                Qn = Nearest(G, Qrand, obstacles)
                Qs = Steer(G, Qn, Qrand, obstacles)
        Qnear = Near(G, Qs, k)
        G.add_vertex(Qs)
        Qp = ChooseParent(G, Qs, Qn, Qnear,obstacles)
        G.add_edge(Qp, Qs)
        Update(G, Qs, Qnear,Qinit,obstacles)

    Qn = Near(G, Qgoal, k)
    G.add_vertex(Qgoal)
    for Q in Qn:
        Qnst=Nearest(G,Qgoal,obstacles)
        if (CollisionFree(Qnst,Qgoal,obstacles)):
            G.add_edge(Qnst, Qgoal)
            Update(G, Qgoal, Near(G, Qgoal, k), Qinit, obstacles)
            break
        if (CollisionFree(Q,Qgoal,obstacles)):
            G.add_edge(Q, Qgoal)
            Update(G, Qgoal, Near(G, Qgoal, k), Qinit, obstacles)
            break


    path_vertices = shortest_path(G, Qinit, Qgoal)

    #path = []
    #for i in range(len(path_vertices) - 1):
        #path.append([path_vertices[i], path_vertices[i + 1]])
    #G.plot_edges(obstacles,path)
    #plt.show()

    return path_vertices,G

