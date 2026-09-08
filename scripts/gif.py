import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
import imageio

# 1. WCZYTANIE DANYCH
cloud_orig = o3d.io.read_point_cloud("data/table_scene_lms400.ply")
cloud_down = o3d.io.read_point_cloud("data/table_scene_lms400_voxel_downsampled.ply")
cloud_sor = o3d.io.read_point_cloud("data/table_scene_lms400_sor_filtered.ply")
cloud_table = o3d.io.read_point_cloud("data/table_scene_lms400_ransac_table.ply")
cloud_objects = o3d.io.read_point_cloud("data/table_scene_lms400_ransac_objects.ply")
target_cloud = o3d.io.read_point_cloud("data/cluster_above_the_table_0.ply")

cluster_clouds = []
cluster_colors = [[0., 0., 1.], [1., 1., 0.], [1., 0., 1.], [0., 1., 1.], [1., 0.5, 0.]]
i = 0
while os.path.exists(f"data/cluster_{i}.ply"):
    cluster = o3d.io.read_point_cloud(f"data/cluster_{i}.ply")
    cluster.paint_uniform_color(cluster_colors[i % len(cluster_colors)])
    cluster_clouds.append(cluster)
    i += 1

# 2. KOLOROWANIE I GEOMETRIA
def color_by_z(pcd):
    points = np.asarray(pcd.points)
    if len(points) == 0: return pcd
    z_vals = points[:, 2]
    z_min, z_max = z_vals.min(), z_vals.max()
    normalized_z = (z_vals - z_min) / (z_max - z_min + 1e-5)
    colors = plt.colormaps['jet'](normalized_z)[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

cloud_orig = color_by_z(cloud_orig)
cloud_down = color_by_z(cloud_down)
cloud_sor = color_by_z(cloud_sor)

cloud_table.paint_uniform_color([0.8, 0.0, 0.0])   
cloud_objects.paint_uniform_color([0.0, 0.8, 0.0]) 
target_cloud.paint_uniform_color([0.0, 1.0, 0.0]) 

obb_center = np.array([-0.164993, 0.300963, -1.31584])
obb_R = np.array([
    [ 0.0651143, -0.997439, -0.0295933],
    [-0.475721,  -0.0570977, 0.877741 ],
    [ 0.877183,   0.0430753, 0.478221 ]
])
obb_extent = np.array([0.0609315, 0.081746, 0.162494])
obb = o3d.geometry.OrientedBoundingBox(obb_center, obb_R, obb_extent)
obb.color = (1.0, 0.5, 0.0)

tcp_position = np.array([-0.166806, 0.354722, -1.28655])
tcp_R = np.array([
    [ 0.0651142,  0.997439, -0.0295933],
    [-0.475721,   0.0570977, 0.877741 ],
    [ 0.877183,  -0.0430753, 0.478221 ]
])
gripper_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.06, origin=[0, 0, 0])
gripper_frame.rotate(tcp_R, center=(0, 0, 0))
gripper_frame.translate(tcp_position)

# 3. STORYBOARD 
scenes = [
    {"name": "1. Oryginal", "geoms": [cloud_orig], "zoom": False},
    {"name": "2. Voxel", "geoms": [cloud_down], "zoom": False},
    {"name": "3. SOR", "geoms": [cloud_sor], "zoom": False},
    {"name": "4. RANSAC", "geoms": [cloud_table, cloud_objects], "zoom": False},
    {"name": "5. Clusters", "geoms": cluster_clouds, "zoom": False},
    {"name": "6. Target (Wyizolowany)", "geoms": [target_cloud], "zoom": True},
    {"name": "7. Target + OBB", "geoms": [target_cloud, obb], "zoom": True},
    {"name": "8. Target + OBB + Grasp", "geoms": [target_cloud, obb, gripper_frame], "zoom": True}
]

# 4. FUNKCJA RENDERUJACA
def generate_gif(filename, rotate=False):
    print(f"\n--- Generowanie: {filename} ---")
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f"Caldera3D - {filename}", width=1280, height=720, visible=True)

    frames = []
    
    for scene_idx, scene in enumerate(scenes):
        print(f"Tworze kadr: {scene['name']}")
        vis.clear_geometries()
        
        for geom in scene["geoms"]:
            vis.add_geometry(geom, reset_bounding_box=(scene_idx == 0))
            
        ctr = vis.get_view_control()
        frames_per_scene = 25 
        
        if scene["zoom"]:
            ctr.set_lookat(obb_center)
            ctr.set_zoom(0.4) 
            frames_per_scene = 40 

        for frame_idx in range(frames_per_scene):
            if rotate:
                ctr.rotate(5.0, 0.0)

            vis.poll_events()
            vis.update_renderer()

            img = vis.capture_screen_float_buffer(False)
            img = (np.asarray(img) * 255).astype(np.uint8)
            frames.append(img)

    vis.destroy_window()
    print(f"Zapisywanie {filename} (15 FPS)...")
    imageio.mimsave(filename, frames, fps=15)

# 5. ODPALENIE GENERATOROW
generate_gif("caldera3d_static.gif", rotate=False)
generate_gif("caldera3d_dynamic.gif", rotate=True)

print("\nGotowe :)")