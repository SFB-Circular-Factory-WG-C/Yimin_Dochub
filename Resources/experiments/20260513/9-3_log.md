## grasp pose
```
header:
  stamp:
    sec: 1778682027
    nanosec: 751138359
  frame_id: base_link
pose:
  position:
    x: -0.8096830248832703
    y: -0.1433899998664856
    z: 0.14614886045455933
  orientation:
    x: -0.9869622327763037
    y: 0.027125038949423336
    z: 0.1585899935670241
    w: 0.004358586424166202
```
## placement pose
```
header:
  stamp:
    sec: 1778682027
    nanosec: 755471664
  frame_id: placement_link
pose:
  position:
    x: -0.028006663089498035
    y: 0.010751291709686694
    z: -0.020996300064914264
  orientation:
    x: 0.7064544753026966
    y: 0.689747993640967
    z: -0.11351660576043641
    w: 0.11083212444811129
```
## vmf python log
```
[INFO] [1778682014.700045786] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778682014.802155927] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778682014.901254858] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778682014.902725918] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778682014, nanosec=760642453), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923620495319472, y=0.1334674538411843, z=0.6857876130736786), rotation=geometry_msgs.msg.Quaternion(x=0.6866912037537055, y=0.7247927369608989, z=-0.05593168187223586, w=-0.0015251553886205032)))
[INFO] [1778682014.907300704] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 9-3
Bounding Box center: [-0.12082538 -0.16770129  0.05906765]
Point-cloud center of gravity: [-0.12177031 -0.17439812  0.07322267]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/9-3_poses.ply
[INFO] [1778682027.746407056] [pcd_subsriber_node]: Inference time: 4.972s
[INFO] [1778682027.750814188] [pcd_subsriber_node]: Predicted translation: [-0.809683   -0.14339     0.14614886], quaternion: (-0.9869622327763037, 0.027125038949423336, 0.1585899935670241, 0.004358586424166202)
[INFO] [1778682027.752178972] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[ 2.27233702e-02  9.99713729e-01 -7.49615735e-03 -2.80066631e-02]
 [ 9.49388510e-01 -2.39278933e-02 -3.13191426e-01  1.07512917e-02]
 [-3.13281089e-01 -1.02949194e-09 -9.49660480e-01 -2.09963001e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778682027.755310602] [pcd_subsriber_node]: Placement pose translation: [-0.02800666  0.01075129 -0.0209963 ], quaternion: (0.7064544753026966, 0.689747993640967, -0.11351660576043641, 0.11083212444811129)
[INFO] [1778682027.756386864] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/9-3_grasp.ply
[INFO] [1778682030.963311056] [pcd_subsriber_node]: Inference complete. Shutting down.
```