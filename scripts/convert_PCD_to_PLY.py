# .pcd to .ply file conversion using the open3d library
# just cause i cant find a .ply version of the dataset i wanna use 

import open3d as o3d

pcd = o3d.io.read_point_cloud("c:/Users/Tomas/Documents/Caldera3D/data/table_scene_lms400.pcd")

print(f"Read {len(pcd.points)} points. Saving into clean ASCII .ply")
o3d.io.write_point_cloud("c:/Users/Tomas/Documents/Caldera3D/data/table_scene_lms400.ply", pcd, write_ascii=True)

print("[SUCCESS] Yay :)")