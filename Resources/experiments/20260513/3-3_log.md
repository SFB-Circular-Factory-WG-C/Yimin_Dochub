## grasp pose
```
header:
  stamp:
    sec: 1778677977
    nanosec: 110125982
  frame_id: base_link
pose:
  position:
    x: -0.8284133672714233
    y: 0.1434457004070282
    z: 0.16031737625598907
  orientation:
    x: -0.9975297226616733
    y: -0.032714080148333005
    z: 0.0038292229634353456
    w: 0.06204497093294539
```
## placement pose
```
header:
  stamp:
    sec: 1778677977
    nanosec: 114958571
  frame_id: placement_link
pose:
  position:
    x: -0.0012187604643104782
    y: -0.03428512810936335
    z: 0.008672394879026962
  orientation:
    x: -0.5654839666170798
    y: -0.8224133034230283
    z: 0.05221759483364203
    w: 0.03372780220566522
```
## vmf python log
```
[INFO] [1778677961.080784799] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778677961.182923081] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778677961.291925181] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778677961.293398633] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778677961, nanosec=158839048), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923467343536729, y=0.13349683611082888, z=0.6857938611989274), rotation=geometry_msgs.msg.Quaternion(x=0.6867044954588232, y=0.7247814909426049, z=-0.05591350637073401, w=-0.001551163752748189)))
[INFO] [1778677961.297905166] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 3-3
Bounding Box center: [-0.09204454  0.12083381  0.03968262]
Point-cloud center of gravity: [-0.10000688  0.14343062  0.06160734]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/3-3_poses.ply
[INFO] [1778677977.106712345] [pcd_subsriber_node]: Inference time: 6.732s
[INFO] [1778677977.109808660] [pcd_subsriber_node]: Predicted translation: [-0.82841337  0.1434457   0.16031738], quaternion: (-0.9975297226616733, -0.032714080148333005, 0.0038292229634353456, 0.06204497093294539)
[INFO] [1778677977.111166492] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-0.35818067  0.92660069 -0.11453281 -0.00121876]
 [ 0.93364552  0.35500241 -0.04774383 -0.03428513]
 [-0.00358004 -0.12403395 -0.99227154  0.00867239]
 [ 0.          0.          0.          1.        ]]
[INFO] [1778677977.114773702] [pcd_subsriber_node]: Placement pose translation: [-0.00121876 -0.03428513  0.00867239], quaternion: (-0.5654839666170798, -0.8224133034230283, 0.05221759483364203, 0.03372780220566522)
[INFO] [1778677977.116045803] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/3-3_grasp.ply
[INFO] [1778677979.364220942] [pcd_subsriber_node]: Inference complete. Shutting down.
```