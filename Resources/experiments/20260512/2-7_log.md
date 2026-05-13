## grasp pose
```
header:
  stamp:
    sec: 1778603129
    nanosec: 575264286
  frame_id: base_link
pose:
  position:
    x: -0.6667931079864502
    y: 0.15616026520729065
    z: 0.1624603569507599
  orientation:
    x: -0.9916835892616322
    y: -0.05301101720775195
    z: 0.1031626848404847
    w: 0.05577590250519345
```
## placement pose
```
header:
  stamp:
    sec: 1778603129
    nanosec: 579944371
  frame_id: placement_link
pose:
  position:
    x: -0.0030332862181245146
    y: 0.007072945014818144
    z: 0.004242961891879782
  orientation:
    x: 0.6819551057827685
    y: 0.721930566945069
    z: -0.11318711608110535
    w: 0.030694738318257114
```
## vmf python log
```
[INFO] [1778603120.358568088] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778603120.460985420] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778603120.562448001] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778603120.563905872] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778603120, nanosec=420844443), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923742046694829, y=0.13345469049316924, z=0.6858599478262511), rotation=geometry_msgs.msg.Quaternion(x=0.6866648520733527, y=0.7248150005278927, z=-0.055964929331144574, w=-0.0015882763586503104)))
[INFO] [1778603120.568398398] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 2-7
Bounding Box center: [0.02720134 0.13397936 0.05349204]
Point-cloud center of gravity: [0.02538659 0.1715769  0.06094275]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/2-7_poses.ply
[INFO] [1778603129.570931476] [pcd_subsriber_node]: Inference time: 5.402s
[INFO] [1778603129.574937396] [pcd_subsriber_node]: Predicted translation: [-0.6667931   0.15616027  0.16246036], quaternion: (-0.9916835892616322, -0.05301101720775195, 0.1031626848404847, 0.05577590250519345)
[INFO] [1778603129.576529990] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-0.06799013  0.99159702 -0.11005813 -0.00303329]
 [ 0.97769997  0.04425182 -0.20529134  0.00707295]
 [-0.198696   -0.12156162 -0.97249305  0.00424296]
 [ 0.          0.          0.          1.        ]]
[INFO] [1778603129.579719595] [pcd_subsriber_node]: Placement pose translation: [-0.00303329  0.00707295  0.00424296], quaternion: (0.6819551057827685, 0.721930566945069, -0.11318711608110535, 0.030694738318257114)
[INFO] [1778603129.581016981] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/2-7_grasp.ply
[INFO] [1778603132.228657690] [pcd_subsriber_node]: Inference complete. Shutting down.
```