import open3d as o3d
import numpy as np

print("Uruchamiam widok detalu z pozycją TCP (Tool Center Point)...")

# 1 Wczytanie tylko docelowego klastra 
target_cloud = o3d.io.read_point_cloud("data/cluster_above_the_table_0.ply")
target_cloud.paint_uniform_color([0.7, 0.7, 0.7]) # Jasnoszary

# 2 OBB
obb_center = np.array([-0.164993, 0.300963, -1.31584])
obb_R = np.array([
    [ 0.0651143, -0.997439, -0.0295933],
    [-0.475721,  -0.0570977, 0.877741 ],
    [ 0.877183,   0.0430753, 0.478221 ]
])
obb_extent = np.array([0.0609315, 0.081746, 0.162494])
obb = o3d.geometry.OrientedBoundingBox(obb_center, obb_R, obb_extent)
obb.color = (1.0, 0.5, 0.0) # Pomarańczowe linie pudelka

# 3 WIZUALIZACJA GRASP POSE 
tcp_position = np.array([-0.166806, 0.354722, -1.28655])
tcp_R = np.array([
    [ 0.0651142,  0.997439, -0.0295933],
    [-0.475721,   0.0570977, 0.877741 ],
    [ 0.877183,  -0.0430753, 0.478221 ]
])

gripper_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.06, origin=[0, 0, 0])
gripper_frame.rotate(tcp_R, center=(0, 0, 0))
gripper_frame.translate(tcp_position)

# 4 Finalna wizualizacja
geometries = [target_cloud, obb, gripper_frame]

print("\n--- LEGENDA OSI (Standard Robotyki) ---")
print(" CZERWONY (X) : Kierunek zacisku szczęk (docelowy rozstaw: 61 mm)")
print(" ZIELONY  (Y) : Oś wzdłuż dłuższego boku puszki")
print(" NIEBIESKI(Z) : Kierunek podejścia/odejścia robota (do góry)")
print(" ŚRODEK OSI   : Punkt TCP (zanurzony 2 cm w paczkę)")

o3d.visualization.draw_geometries(
    geometries,
    window_name="Caldera3D - Grasp Pose (Close Up)",
    width=1200,
    height=900,
    mesh_show_back_face=True 
)