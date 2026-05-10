## grasp pose
```
header:
  stamp:
    sec: 1778250785
    nanosec: 554658274
  frame_id: base_link
pose:
  position:
    x: -0.8063430786132812
    y: 0.1419442892074585
    z: 0.14245052635669708
  orientation:
    x: 0.9774913809351331
    y: 0.10563461846660172
    z: -0.10704219603152725
    w: 0.14796586041276635
```
## placement pose
```
missing
```
## vmf python log
```
[INFO] [1778250758.809485475] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778250758.911554047] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778250759.004571457] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778250759.006060837] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778250758, nanosec=952443385), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923550639806142, y=0.1334166561574658, z=0.685810860311551), rotation=geometry_msgs.msg.Quaternion(x=0.6866713145174372, y=0.7248090466700917, z=-0.05596467186197093, w=-0.001518942878944413)))
[INFO] [1778250759.008678116] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [-0.12506078  0.11394138  0.04803454]
Point-cloud center of gravity: [-0.11451484  0.10745853  0.06482438]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778250785.547084811] [pcd_subsriber_node]: Inference time: 26.536s
[INFO] [1778250785.554276408] [pcd_subsriber_node]: Predicted translation: [-0.8063431   0.14194429  0.14245053], quaternion: (0.9774913809351331, 0.10563461846660172, -0.10704219603152725, 0.14796586041276635)
[INFO] [1778250785.555914432] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-0.11207081  0.94746409  0.29958654 -0.03029896]
 [ 0.96415109  0.17665371 -0.19800567  0.01545775]
 [-0.24052629  0.26665601 -0.93329614 -0.01497894]
 [ 0.          0.          0.          1.        ]]
[INFO] [1778250785.562109333] [pcd_subsriber_node]: Placement pose translation: [-0.03029896  0.01545775 -0.01497894], quaternion: (0.6412042687245126, 0.7453221834752294, 0.023027072477830465, 0.18116755372383891)
[INFO] [1778250785.563514903] [pcd_subsriber_node]: Place pose published.
[INFO] [1778250880.865659572] [pcd_subsriber_node]: Inference complete. Shutting down.
```
## moveit
```
[moveit2_iface-3] [ERROR] [1778250506.321224169] [moveit2_iface]: IK solution not found for target pose.
[moveit2_iface-3] [INFO] [1778250506.423615771] [moveit2_iface]: Received new target pose for servo control.
[moveit2_iface-3] [WARN] [1778250506.424110058] [moveit2_iface]: IK solution found but is in collision or invalid.
[moveit2_iface-3] [WARN] [1778250506.424256904] [moveit2_iface]: Colliding links: forearm_link, orbbec_femto_mega_link
```