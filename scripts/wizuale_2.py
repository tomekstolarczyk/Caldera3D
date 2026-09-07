import open3d as o3d
import numpy as np 
import matplotlib.pyplot as plt
import os 

# 1 load data
cloud_orig = o3d.io.read_point_cloud("data/table_scene_lms400.ply")
final_clusters = []
i = 0
while os.path.exists(f"data/cluster_above_the_table_{i}.ply"):
    cluster = o3d.io.read_point_cloud(f"data/cluster_above_the_table_{i}.ply")
    cluster.paint_uniform_color([0.0, 1.0, 0.0])
    final_clusters.append(cluster)
    i += 1

# 2 coloring
def color_by_z(pcd):
    points = np.asarray(pcd.points)
    if len(points) == 0:
        return pcd
    z_vals = points[:, 2]
    z_min, z_max = z_vals.min(), z_vals.max()
    normalized_z = (z_vals - z_min) / (z_max - z_min + 1e-5)
    cmap = plt.colormaps['jet']
    colors = cmap(normalized_z)[:, :3] 
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd
cloud_orig = color_by_z(cloud_orig)

# 3 visu offset
bbox = cloud_orig.get_axis_aligned_bounding_box()
extent_x = bbox.get_extent()[0]
offset = extent_x + 0.2
for cluster in final_clusters:
    cluster.translate([offset, 0.0, 0.0])

# bounding box (values fixed for now)
center = np.array([-0.16493, 0.304941, -1.31351])
R = np.array([
    [ 0.0727719, -0.996574,  -0.0392906],
    [-0.470329,  -0.0690306,  0.879787],
    [ 0.879486,   0.0455443,  0.473741]
])
extent = np.array([0.0610219, 0.0825348, 0.155759])
obb = o3d.geometry.OrientedBoundingBox(center, R, extent)
obb.color = (1.0, 0.0, 0.0) # red
obb.translate([offset, 0.0, 0.0])

# 5 final visu
print("Wizualizacja: [Oryginał] -> [Wyizolowana paczka z OBB]")
geometries = [cloud_orig] + final_clusters + [obb]
o3d.visualization.draw_geometries(
    geometries,
    window_name="Caldera3D - Wyizolowany Target",
    width=1920,
    height=800
)