import open3d as o3d
import numpy as np

# Load the OBJ file
mesh = o3d.io.read_triangle_mesh("../blender/N3OpenArea.obj")

# Load the path data
path = np.genfromtxt("../path_dense.csv", delimiter=",", skip_header=1)

# Extract the positions from the path data
positions = path[:, 1:4] # 3:6 for path 1:4 for path_dense

# Create a point cloud from the positions
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(positions)

# Visualize the mesh and the point cloud
o3d.visualization.draw_geometries([mesh, pcd])
