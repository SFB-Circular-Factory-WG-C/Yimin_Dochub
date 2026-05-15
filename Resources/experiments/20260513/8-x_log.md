## grasp pose
```
header:
  stamp:
    sec: 1778680760
    nanosec: 374785202
  frame_id: base_link
pose:
  position:
    x: -0.6704888343811035
    y: -0.21018099784851074
    z: 0.14662303030490875
  orientation:
    x: 0.9834344123752964
    y: 0.035318676788306136
    z: -0.17767563375447798
    w: 0.006380971393387073
```
## placement pose
```
header:
  stamp:
    sec: 1778680760
    nanosec: 392177080
  frame_id: placement_link
pose:
  position:
    x: 0.000513824812193503
    y: 0.045240009835136985
    z: -0.012795812816868923
  orientation:
    x: 0.6571560023946124
    y: 0.7324866145627374
    z: -0.11872740020107461
    w: 0.13233726778442162
```
## vmf python log
```
[INFO] [1778680748.591992012] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778680748.693991952] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778680748.798289080] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778680748.799773221] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778680748, nanosec=684863319), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923553839404075, y=0.13342021387111852, z=0.6858382670698565), rotation=geometry_msgs.msg.Quaternion(x=0.686648559225525, y=0.7248304713224288, z=-0.05596552030704348, w=-0.0015506426984374117)))
[INFO] [1778680748.804399439] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 8-3
Bounding Box center: [-0.04974823 -0.20476835  0.04417095]
Point-cloud center of gravity: [ 0.01838837 -0.21126355  0.07266673]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/8-3_poses.ply
[INFO] [1778680760.371003425] [pcd_subsriber_node]: Inference time: 6.659s
[INFO] [1778680760.374471593] [pcd_subsriber_node]: Predicted translation: [-0.67048883 -0.210181    0.14662303], quaternion: (0.9834344123752964, 0.035318676788306136, -0.17767563375447798, 0.006380971393387073)
[INFO] [1778680760.375833777] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-1.01265676e-01  9.94140149e-01  3.78257104e-02  5.13824812e-04]
 [ 9.31291866e-01  1.08099593e-01 -3.47864945e-01  4.52400098e-02]
 [-3.49915415e-01 -2.72870659e-09 -9.36781347e-01 -1.27958128e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778680760.391878575] [pcd_subsriber_node]: Placement pose translation: [ 0.00051382  0.04524001 -0.01279581], quaternion: (0.6571560023946124, 0.7324866145627374, -0.11872740020107461, 0.13233726778442162)
[INFO] [1778680760.393273593] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/8-3_grasp.ply
[INFO] [1778680762.731235952] [pcd_subsriber_node]: Inference complete. Shutting down.
```