#
import open3d as o3d
import numpy as np
print("Load a ply point cloud, print it, and render it")
ply_point_cloud = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud('/home/ahamza/Downloads/lm_models/models_eval/obj_000008.ply')
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd],
                                zoom=0.3412,
                                front=[0.4257, -0.2125, -0.8795],
                                lookat=[2.6172, 2.0475, 1.532],
                                up=[-0.0694, -0.9768, 0.2024])

# # create point cloud from np array [N, 3]
#     o3d_pcd_a = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd_xyz)) 
#     # add colors from np array [N, 3] (I don't remember if they have to be floating points or integers)
#     o3d_pcd_a.color = o3d.utility.Vector3dVector(pcd_rgb) 
#     # or alternatively add a uniform color to the point cloud
#     o3d_pcd_a.paint_uniform_color([1.,0.,0.])
#     # this creates a set of axis to visualize the origin of the space, can be useful in some cases
#     mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0., 0., 0.])
#     # this function will visualize all the point clouds passed as a list
#     o3d.visualization.draw_geometries([o3d_pcd_a, mesh])








# import torch

# # Load the model parameters
# model_params = torch.load('/home/ahamza/WaffleIron/pretrained_models/WaffleIron-48-256__kitti/ckpt_last.pth', map_location='cpu')

# # Access the 'net' key and iterate through its parameters
# net_params = model_params['net']
# for name, param in net_params.items():
#     if isinstance(param, torch.Tensor):
#         print(f"Parameter name: {name}, Size: {param.size()}")
#     else:
#         # Handle cases where the parameter is not a tensor (e.g., nested dictionaries)
#         print(f"Parameter name: {name}, Type: {type(param)}")


