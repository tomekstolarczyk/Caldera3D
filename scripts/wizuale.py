import open3d as o3d
import numpy as np 
import matplotlib.pyplot as plt

# 1 data
cloud_orig = o3d.io.read_point_cloud("data/table_scene_lms400.ply")
cloud_down = o3d.io.read_point_cloud("data/table_scene_lms400_voxel_downsampled.ply")
cloud_sor = o3d.io.read_point_cloud("data/table_scene_lms400_sor_filtered.ply")
cloud_table = o3d.io.read_point_cloud("data/table_scene_lms400_ransac_table.ply")
cloud_objects = o3d.io.read_point_cloud("data/table_scene_lms400_ransac_objects.ply")

# 2 colors - first three
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

# ransac colors
cloud_table.paint_uniform_color([1.0, 0.0, 0.0])    # plaszyczna czerwony
cloud_objects.paint_uniform_color([0.0, 1.0, 0.0])  # Obiekty zielony

# 3 visu placement
bbox = cloud_orig.get_axis_aligned_bounding_box()
extent_x = bbox.get_extent()[0]
offset = extent_x + 0.2

cloud_orig.translate([-offset, 0.0, 0.0]) # oryginal lewo
# downsampled voxel grid zostaje na srodku (0.0, 0.0, 0.0)
cloud_sor.translate([offset, 0.0, 0.0]) # SOR w prawo
# RANSAC (obie chmury nakladamy na siebie i przesuwamy jeszcze bardziej w prawo)
cloud_table.translate([offset * 2, 0.0, 0.0])
cloud_objects.translate([offset * 2, 0.0, 0.0])

# 4 Faktyczne visu
print("Wizualizacja: [Oryginał] -> [Voxel Downsampled] -> [SOR Filtered] -> [RANSAC Segmentation]")
o3d.visualization.draw_geometries(
    [cloud_orig, cloud_down, cloud_sor, cloud_table, cloud_objects],
    window_name="Caldera3D Pipeline",
    width=1920,
    height=800
)