import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import Delaunay
import itertools
import time

def circumscribed_sphere_radius(tet):
    A, B, C, D = tet
    M = np.vstack([B - A, C - A, D - A])
    b = np.array([np.dot(B, B) - np.dot(A, A),
                  np.dot(C, C) - np.dot(A, A),
                  np.dot(D, D) - np.dot(A, A)])
    try:
        center = np.linalg.solve(2 * M, b)
    except np.linalg.LinAlgError:
        return np.inf
    radius = np.linalg.norm(A - center)
    return radius

def alpha_shape_3D(points, alpha):
    if len(points) < 4:
        return []

    delaunay = Delaunay(points)
    tetrahedra = points[delaunay.simplices]
    faces = {}

    for i, tet in enumerate(tetrahedra):
        R = circumscribed_sphere_radius(tet)
        if R < alpha:
            inds = delaunay.simplices[i]
            for face in itertools.combinations(inds, 3):
                face = tuple(sorted(face))
                faces[face] = faces.get(face, 0) + 1
    
    boundary_faces = [face for face, count in faces.items() if count == 1]
    return boundary_faces

def main():
    # Parameters
    max_points = 128
    alpha = 2.0  # Adjust alpha to get one unified concave hull.
    
    points = []
    
    plt.ion()
    fig = plt.figure(figsize=(8, 6), facecolor='none')
    ax = fig.add_subplot(111, projection='3d', facecolor='none')
    
    fig.patch.set_alpha(0)
    ax.patch.set_alpha(0)
    
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    fig.suptitle("3D Points and Their Alpha Shape (Concave Hull)", fontsize=14)
    
    fig.text(0.5, 0.01, "MIT License Copyright (c) 2025 Hikmet Ozdemir", ha='center', fontsize=10)
    
    azim_angle = 0

    for i in range(max_points):
        new_point = np.random.uniform(-1, 1, size=3)
        points.append(new_point)
        pts = np.array(points)
        
        ax.cla()
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f"n = {len(points)}", fontsize=10)
        
        ax.view_init(elev=30, azim=azim_angle)
        azim_angle = (azim_angle + 2) % 360
        
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color='blue', s=20)
        
        if len(points) >= 4:
            faces = alpha_shape_3D(pts, alpha)
            triangles = [pts[list(face)] for face in faces]
            if triangles:
                poly = Poly3DCollection(triangles, facecolors='cyan', edgecolors='r', alpha=0.5)
                ax.add_collection3d(poly)
        
        plt.draw()
        plt.pause(0.01)
    
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    main()
