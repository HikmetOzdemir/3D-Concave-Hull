import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import Delaunay
import itertools
import time

def circumscribed_sphere_radius(tet):
    """
    Given a tetrahedron (an array of 4 points in 3D), compute the radius
    of its circumscribed sphere.
    If the tetrahedron is degenerate, return np.inf.
    """
    A, B, C, D = tet  # unpack the four vertices
    # We find the center by solving for x in:
    #   |A - x|^2 = |B - x|^2 = |C - x|^2 = |D - x|^2
    # Subtract the equation for A from that for B, C, D.
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
    """
    Compute the 3D alpha shape (concave hull) of a set of 3D points.
    Returns a list of faces (each face is a tuple of three point indices into the points array)
    that make up the boundary of the alpha shape.
    """
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
                if face in faces:
                    faces[face] += 1
                else:
                    faces[face] = 1
    
    boundary_faces = [face for face, count in faces.items() if count == 1]
    return boundary_faces

def main():
    # Parameters
    max_points = 128
    alpha = 0.3  # You may need to adjust this parameter for different point distributions.
    
    points = []
    
    plt.ion()
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("3D Points and Their Alpha Shape (Concave Hull)")
    
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
        ax.set_title(f"3D Points and Their Alpha Shape (n={len(points)})")
        
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color='blue', s=20)
        if len(points) >= 4:
            faces = alpha_shape_3D(pts, alpha)
            triangles = [pts[list(face)] for face in faces]
            if triangles:
                poly = Poly3DCollection(triangles, facecolors='cyan', edgecolors='r', alpha=0.5)
                ax.add_collection3d(poly)
        
        plt.draw()
        plt.pause(0.1)
    
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    main()
