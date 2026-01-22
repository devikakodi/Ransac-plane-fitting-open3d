import open3d as o3d
import numpy as np
import random
import scipy.spatial

def ransac_plane_fitting(point_cloud, distance_threshold=0.025, num_iterations=2000):
    points = np.asarray(point_cloud.points)
    best_inliers = []
    best_plane = None

    for _ in range(num_iterations):
        # Randomly sample three points to form a plane
        sample_points = points[random.sample(range(len(points)), 3)]
        p1, p2, p3 = sample_points

        # Calculate the plane normal vector using cross product
        normal = np.cross(p2 - p1, p3 - p1)
        normal = normal / np.linalg.norm(normal)  # Normalize the normal vector

        # Plane equation ax + by + cz + d = 0
        d = -np.dot(normal, p1)
        plane = np.append(normal, d)

        # Calculate distances of all points to the plane
        distances = np.abs(np.dot(points, normal) + d) / np.linalg.norm(normal)
        
        # Find inliers
        inliers = np.where(distances < distance_threshold)[0]
        
        # Update best plane if more inliers are found
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_plane = plane

    return best_plane, best_inliers

def expand_inliers(point_cloud, inliers, radius=0.03):
    points = np.asarray(point_cloud.points)
    inlier_points = points[inliers]
    kdtree = scipy.spatial.cKDTree(points)
    additional_inliers = []

    for point in inlier_points:
        indices = kdtree.query_ball_point(point, radius)
        additional_inliers.extend(indices)

    expanded_inliers = np.unique(np.concatenate((inliers, additional_inliers)))
    return expanded_inliers

# Load the demo point cloud
pcd = o3d.data.PCDPointCloud()
point_cloud = o3d.io.read_point_cloud(pcd.path)

# Run RANSAC with adjusted parameters
best_plane, best_inliers = ransac_plane_fitting(point_cloud)

# Expand inliers to cover more of the plane area
expanded_inliers = expand_inliers(point_cloud, best_inliers)

# Visualize the plane with expanded inliers
inlier_cloud = point_cloud.select_by_index(expanded_inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])  # Mark inliers in red
outlier_cloud = point_cloud.select_by_index(expanded_inliers, invert=True)

# Display the result
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                  zoom=1,
                                  front=[-0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])