## grasp pose
```
header:
  stamp:
    sec: 1778679308
    nanosec: 413466008
  frame_id: base_link
pose:
  position:
    x: -0.8215228319168091
    y: 0.1673155128955841
    z: 0.1595645695924759
  orientation:
    x: 0.996628237852906
    y: 0.06442489448980329
    z: -0.0507035076464092
    w: 0.003277620560717969
```
## placement pose
```
header:
  stamp:
    sec: 1778679308
    nanosec: 416893033
  frame_id: placement_link
pose:
  position:
    x: -0.04833723167079225
    y: 0.00949518053700571
    z: -0.00843214401580683
  orientation:
    x: 0.6592792756202855
    y: 0.7501794772412618
    z: -0.03354086443815125
    w: 0.0381654172826658
```
## vmf python log
```
[INFO] [1778679299.028475915] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778679299.131193922] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778679299.232748702] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778679299.234220129] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778679299, nanosec=146720823), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923593488588521, y=0.1334406304894235, z=0.6858743463316931), rotation=geometry_msgs.msg.Quaternion(x=0.6866627094807005, y=0.724815939792622, z=-0.055979455484520636, w=-0.0015739742112857052)))
[INFO] [1778679299.238706059] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 3-7
Bounding Box center: [-0.13472368  0.11801074  0.05694918]
Point-cloud center of gravity: [-0.12728348  0.11994015  0.07704425]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/3-7_poses.ply
[INFO] [1778679308.407692144] [pcd_subsriber_node]: Inference time: 4.112s
[INFO] [1778679308.413141584] [pcd_subsriber_node]: Predicted translation: [-0.82152283  0.16731551  0.15956457], quaternion: (0.996628237852906, 0.06442489448980329, -0.0507035076464092, 0.003277620560717969)
[INFO] [1778679308.414481648] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-1.27788477e-01  9.91715851e-01  1.30362320e-02 -4.83372317e-02]
 [ 9.86595375e-01  1.28451704e-01 -1.00646674e-01  9.49518054e-03]
 [-1.01487420e-01  1.11684140e-09 -9.94836867e-01 -8.43214402e-03]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778679308.416748837] [pcd_subsriber_node]: Placement pose translation: [-0.04833723  0.00949518 -0.00843214], quaternion: (0.6592792756202855, 0.7501794772412618, -0.03354086443815125, 0.0381654172826658)
[INFO] [1778679308.417846559] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/3-7_grasp.ply
[INFO] [1778679310.720146866] [pcd_subsriber_node]: Inference complete. Shutting down.
```