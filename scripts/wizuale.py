# visualization using open3d library
# nothing much just something easy to be able to see it quickly
# will make sure this becomes more complex and just better in the future 

import open3d as o3d
import numpy as np 
import matplotlib as plt # for color palette only

cloud_orig = o3d.io.read_point_cloud("data/table_scene_lms400.ply")
cloud_down = o3d.io.read_point_cloud("data/table_scene_lms400_downsampled.ply")

def color_by_z(pcd):
    points = np.asarray(pcd.points)
    if len(points) == 0:
        return pcd
    
    z_vals = points[:, 2]
    z_min, z_max = z_vals.min(), z_vals.max()
    normalized_z = (z_vals - z_min) / (z_max - z_min + 1e-5)

    # color gradient from matplotlib
    cmap = plt.colormaps['jet']
    colors = cmap(normalized_z)[:, :3] 
    
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

cloud_orig = color_by_z(cloud_orig)
cloud_down = color_by_z(cloud_down)

bbox = cloud_orig.get_axis_aligned_bounding_box()
extent_x = bbox.get_extent()[0]
cloud_orig.translate([-extent_x - 0.2, 0.0, 0.0])

o3d.visualization.draw_geometries(
    [cloud_orig, cloud_down],
    window_name="Caldera3D",
    width=1280,
    height=720
)