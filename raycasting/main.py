"""
Raytracing
Written by Eraldo Ribeiro, modified by Tyler Gutowski
"""

import numpy as np
from PIL import Image as im

class Ray:
    def __init__(self, eye_position, point_2d):
        self.eye_position = eye_position
        self.point_2d = point_2d
        
    def get_3d_point(self, distance):        
        point_3d = self.eye_position + (self.point_2d - self.eye_position) * distance
        return point_3d

class Sphere:
    def __init__(self, center, radius, color):
        self.center = center
        self.radius = radius
        self.color = color
        
    def intersect(self, ray):
        direction = ray.point_2d - ray.eye_position
        ray_starting_point = ray.eye_position
        center = self.center
        radius = self.radius
        A = np.dot(direction, direction)
        B = 2.0 * np.dot(direction, (ray_starting_point - center))
        C = np.dot((ray_starting_point - center), (ray_starting_point - center)) - radius * radius
        delta = B*B - 4.0 * A * C
        if delta < 0:
            return float("inf")
        else:
            quad1 = (-B - np.sqrt(delta)) / (2.0 * A)
            quad2 = (-B + np.sqrt(delta)) / (2.0 * A)
            distance = np.min([quad1, quad2])
            return distance
            
    def get_normal(self, point_3d):
        normalized = (point_3d - self.center) / np.linalg.norm(point_3d - self.center)
        return normalized

class Plane:
    def __init__(self, point, rotation, color):
        self.point    = point
        self.rotation = rotation 
        self.color    = color
        
    def intersect(self, ray):
        distance = ray.point_2d - ray.eye_position
        eye_position = ray.eye_position
        point = self.point
        rotation = self.rotation
        distance = np.dot((point - eye_position), rotation) / np.dot(distance, rotation)
        if distance < 0:
            return float('inf')  
        return distance
        
    def get_normal(self, p):
        return self.rotation

class Scene: 
    light_source = np.array((.25, .5, 1)).transpose()
    light_source = light_source / np.linalg.norm(light_source)
    
    def __init__(self, camera):
        self.camera = camera
        list = []
        self.scene_objects = []
        
        center = np.array((-40, -100, -500.0)).transpose()
        radius = 50.0
        color = np.array((255, 0, 255)).transpose()
        bottom_segment = Sphere(center, radius, color)
        self.scene_objects.append(bottom_segment)
        
        center = np.array((0, -20, -500.0)).transpose()
        radius = 50.0
        color = np.array((0, 255, 255)).transpose()
        middle_segment = Sphere(center, radius, color)
        self.scene_objects.append(middle_segment)
        
        center = np.array((-40, 60, -500.0)).transpose()
        radius = 50.0
        color = np.array((255, 255, 0)).transpose()
        top_segment = Sphere(center, radius, color)
        self.scene_objects.append(top_segment)

        center = np.array((40, 60, -2000.0)).transpose()
        radius = 800.0
        color = np.array((0, 255, 0)).transpose()
        giant_sphere = Sphere(center, radius, color)
        self.scene_objects.append(giant_sphere)
        
        p1 = np.array((0, -100, -350.0)).transpose()
        n = np.array((np.sin(0), np.cos(0), 0)).transpose()
        color = np.array((255, 255, 255)).transpose()
        ground = Plane(p1, n, color)
        self.scene_objects.append(ground)

        p1 = np.array((0, 0, -10000.0)).transpose()
        n = np.array((np.sin(0), np.cos(0), np.sin(90))).transpose()
        color = np.array((0, 220, 255)).transpose()
        background = Plane(p1, n, color)
        self.scene_objects.append(background)
        
    def find_intersection(self, ray):
        hit_list = []
        for surface in self.scene_objects: 
            distance = surface.intersect(ray)
            if float('inf') != distance:
                p = ray.get_3d_point(distance)
                hitInfo = HitInformation(surface, p, distance)
                hit_list.append(hitInfo)
        return hit_list
        
    def get_color(self, hit_list):  
        pixelcolor = np.array((0.0, 0.0, 0.0))
        distance = float("inf")
        for hit in hit_list:
            normal = hit.object.get_normal(hit.point)
            diffuse_shading = max(0, np.dot(self.light_source, normal))
            ambient_color = hit.object.color * 0.3 
            specular_component = max(0, np.dot(self.light_source, normal)) ** 64
            if hit.distance <= distance:
                distance = hit.distance
                pixelcolor = (ambient_color + hit.object.color * diffuse_shading  + hit.object.color * specular_component) / 2
                for i in range(len(pixelcolor)):
                    pixelcolor[i] = min(255, pixelcolor[i])
        return pixelcolor

class Camera:
    color_channels = 3 # RGB
    eye_position = np.array((0.0, 0.0, 0.0)).transpose()
    
    def __init__(self, f, nrows, ncols):
        self.f = f         
        self.nrows = nrows 
        self.ncols = ncols
        self.I = np.zeros([self.nrows, self.ncols, self.color_channels])
        
    def cast_ray(self, i, j):
        u =  (j + 0.5) - self.ncols/2 
        v = -(i + 0.5) + self.nrows/2
        s = np.array((u, v, -self.f)).transpose()
        ray = Ray(self.eye_position, s)
        return ray
   
class HitInformation:
    def __init__(self, object, point, distance):
        self.object = object
        self.point = point
        self.distance = distance

def generate_image(focal, height, width):
    camera = Camera(focal, height, width)
    scene  = Scene(camera)
    for i in range(height): 
        for j in range(width):
            ray = camera.cast_ray(i, j)
            hit = scene.find_intersection(ray)   
            camera.I[i,j,:] = scene.get_color(hit)
    image = im.fromarray(np.uint8(camera.I))
    image.save('image.png')

def main():
    height = 256
    width  = 256
    focal  = 250
    generate_image(focal, height, width)

main()