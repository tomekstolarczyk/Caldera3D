import open3d as o3d
import numpy as np 
import matplotlib.pyplot as plt
import os 

# 1 load data
cloud_orig = o3d.io.read_point_cloud("data/table_scene_lms400.ply")
cloud_down = o3d.io.read_point_cloud("data/table_scene_lms400_voxel_downsampled.ply")
cloud_sor = o3d.io.read_point_cloud("data/table_scene_lms400_sor_filtered.ply")
cloud_table = o3d.io.read_point_cloud("data/table_scene_lms400_ransac_table.ply")
cloud_objects = o3d.io.read_point_cloud("data/table_scene_lms400_ransac_objects.ply")

# cluster data
cluster_clouds = []
cluster_colors = [
    [0.0, 0.0, 1.0],  
    [1.0, 1.0, 0.0],  
    [1.0, 0.0, 1.0],  
    [0.0, 1.0, 1.0],  
    [1.0, 0.5, 0.0],  
]

i = 0
while os.path.exists(f"data/cluster_{i}.ply"):
    cluster = o3d.io.read_point_cloud(f"data/cluster_{i}.ply")
    color = cluster_colors[i % len(cluster_colors)]
    cluster.paint_uniform_color(color)
    cluster_clouds.append(cluster)
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
cloud_down = color_by_z(cloud_down)
cloud_sor = color_by_z(cloud_sor)

# RANSAC 
cloud_table.paint_uniform_color([1.0, 0.0, 0.0])    # red
cloud_objects.paint_uniform_color([0.0, 1.0, 0.0])  # green

# 3 visu offset
bbox = cloud_orig.get_axis_aligned_bounding_box()
extent_x = bbox.get_extent()[0]
offset = extent_x + 0.2

cloud_orig.translate([-offset, 0.0, 0.0])
cloud_sor.translate([offset, 0.0, 0.0])
cloud_table.translate([offset * 2, 0.0, 0.0])
cloud_objects.translate([offset * 2, 0.0, 0.0])

# clusters
for cluster in cluster_clouds:
    cluster.translate([offset * 3, 0.0, 0.0])




# 4 final visu
print("Wizualizacja: [Oryginał] -> [Voxel] -> [SOR] -> [RANSAC] -> [KLASTRY]")
geometries = [cloud_orig, cloud_down, cloud_sor, cloud_table, cloud_objects] + cluster_clouds
o3d.visualization.draw_geometries(
    geometries,
    window_name="Caldera3D Pipeline",
    width=1920,
    height=800
)