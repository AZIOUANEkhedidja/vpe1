import pygame
import numpy as np
from read_file import *
pygame.init()
w = 200
h = 200
screen = pygame.display.set_mode((w,h))
pygame.display.set_caption("VPE")
focal_length = 10
bg_color = (0, 0, 0)
light_position = np.array([-100,50,50])  
light_intensity = 1.0  
ambient_intensity = 0.3 
class Object3d:
    def __init__(self,surface: pygame.Surface,
                vertices,
                edges, 
                faces,
                position,
                zoom)-> None:
        self.vertices = vertices
        self.edges = edges
        self.faces = faces
        self.halfWidth = surface.get_width() / 2
        self.halfHeight = surface.get_height() / 2
        self.surface = surface
        self.position = position
        self.zoom = zoom

    def draw(self) -> None:
        verts = [[o[u] * 50 + self.position[u] for u in range(3)] for o in self.vertices]        
        for edge in self.edges:
            point1Div = verts[edge[0]][2] / self.zoom
            point2Div = verts[edge[1]][2] / self.zoom
            # x = self.vertices[edge[0]][0]
            # y = self.vertices[edge[0]][1]
            # z = self.vertices[edge[0]][2]
            # point1 = (u0+x/z, u0+y/z)
            # point2 = (x1/z1, y1/z1)
            # use the width of the screen and the hieght to change the point depart of the obj to the center of the screen 
            point1 = [
                verts[edge[0]][0] / point1Div+ self.halfWidth,
                verts[edge[0]][1] / point1Div+ self.halfHeight
            ]
            point2 = [
                verts[edge[1]][0] / point2Div + self.halfWidth,
                verts[edge[1]][1] / point2Div + self.halfHeight
            ]
            pygame.draw.line(self.surface, (111, 111, 111), point1, point2)

class Sphere(Object3d):
    def __init__(self,surface: pygame.Surface, vertices, edges, faces, position,zoom=420) -> None:
        super().__init__(surface,vertices, edges, faces,position,zoom)
        self.center = self.find_center_sphere()
        self.radius = self.find_sphere_radius()
        
    def find_center_sphere(self):
        x = 0
        y = 0
        z = 0
        for vertex in self.vertices:
            x += vertex[0]
            y += vertex[1]
            z += vertex[2]
        return [x / len(self.vertices), y / len(self.vertices), z / len(self.vertices)]


    def find_sphere_radius(self):
        center = self.center
        r = 0
        for v in self.vertices:
            d = np.sqrt((v[0] - center[0]) ** 2 + (v[1] - center[1]) ** 2 + (v[2] - center[2]) ** 2)
            if d > r:
                r = d
        return r


    def intersect(self, ray_origin, ray_direction):
        # oc vector from the camira to the sphere
        oc = np.array(ray_origin) - np.array(self.center)
        # we normalize the ray direction so d*d is 1
        # a = d*d
        a = 1
        b = 2 *np.dot(oc, ray_direction)
        c = np.dot(oc, oc) - self.radius**2
        # equation : a*t^2 + b*t + c = 0
        # if t : there is an intersection else none
        delta = b**2 - 4*a*c
        if delta < 0:
            return None
        else:
            t1 = (-b - np.sqrt(delta)) / (2*a)
            t2 = (-b + np.sqrt(delta)) / (2*a)
            return min(t for t in [t1, t2] if t > 0)
class Camera:
    def __init__(self, position, w, h, f) -> None:
        self.position = position
        self.w = w
        self.h = h
        self.f = f
    def pixel_to_ndc(self,x,y):
        # i choose point (0,0) in the center of the screen 
        # the direction is right and up
        x_ndc = (2*x/self.w) - 1
        y_ndc = 1 - (2*y/self.h)
        return np.array([x_ndc, y_ndc])

    def get_ray(self, x, y):
        ndc = self.pixel_to_ndc(x, y)
        ray = np.array([ndc[0], ndc[1], -self.f])
        ray = ray / np.linalg.norm(ray)
        return self.position, ray

obj_sphere = OBJLoader("sphere.obj")
camira = Camera([0, 0, 200], w, h, focal_length)
sphere = Sphere(screen,obj_sphere.vertices,create_edges_from_faces(obj_sphere.faces),obj_sphere.faces , camira.position,50)

def compute_lambertian_lighting(normal, light_pos, point):
    light_dir = light_pos - point
    light_dir = light_dir / np.linalg.norm(light_dir)
    angle = np.dot(normal, light_dir)
    angle = max(0, angle)  
    gradient_intensity = ambient_intensity + (angle * light_intensity)
    gradient_intensity = np.clip(gradient_intensity, 0, 1)
    return gradient_intensity

running = True
done = False
while running:
    screen.fill(bg_color)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    while not done:
        for x in range(h):
            for y in range(w):
                ray_origin, ray_direction = camira.get_ray(y,x)
                t = sphere.intersect(ray_origin, ray_direction)
                if t is not None:
                    point = ray_origin + t * ray_direction
                    normal = point - sphere.center
                    normal = normal / np.linalg.norm(normal)
                    intensity = compute_lambertian_lighting(normal, light_position, point)
                    gray_value = int(intensity * 255)
                    color = (gray_value, gray_value, gray_value)
                else:
                    color = (111,111,111)
                screen.set_at((y,x), color)
                pygame.display.update()
        done = True

exit()