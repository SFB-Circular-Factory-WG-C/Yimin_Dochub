## grasp pose
```
header:
  stamp:
    sec: 1778256023
    nanosec: 794876212
  frame_id: base_link
pose:
  position:
    x: -0.8263863921165466
    y: -0.04352772235870361
    z: 0.1486140638589859
  orientation:
    x: 0.9840625426177848
    y: 0.010843592241436995
    z: -0.17748099605271073
    w: 0.0019557004013171756
```
## placement pose
```
header:
  stamp:
    sec: 1778256023
    nanosec: 804350813
  frame_id: placement_link
pose:
  position:
    x: -0.005174392146966619
    y: -0.0245288964620487
    z: -0.0053298721876527955
  orientation:
    x: 0.6553334238547928
    y: 0.7341898753954419
    z: -0.11819292680017879
    w: 0.13241511431852335
```
## vmf python log
```
[INFO] [1778256003.042153264] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778256003.144278991] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778256003.237222670] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778256003.238682268] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778256003, nanosec=132487901), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923545564678805, y=0.13346235639637474, z=0.685859087919535), rotation=geometry_msgs.msg.Quaternion(x=0.686685716408895, y=0.7247964950281004, z=-0.055948780842612233, w=-0.00158164383122014)))
[INFO] [1778256003.241228108] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [-0.08929887 -0.07083209  0.0470632 ]
Point-cloud center of gravity: [-0.1155646  -0.03100382  0.05882467]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778256023.787030783] [pcd_subsriber_node]: Inference time: 20.542s
[INFO] [1778256023.794516041] [pcd_subsriber_node]: Predicted translation: [-0.8263864  -0.04352772  0.14861406], quaternion: (0.9840625426177848, 0.010843592241436995, -0.17748099605271073, 0.0019557004013171756)
[INFO] [1778256023.796923380] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-1.06008685e-01  9.93579436e-01  3.95241243e-02 -5.17439215e-03]
 [ 9.30977298e-01  1.13137076e-01 -3.47104221e-01 -2.45288965e-02]
 [-3.49347234e-01  1.58539848e-10 -9.36993361e-01 -5.32987219e-03]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778256023.804082741] [pcd_subsriber_node]: Placement pose translation: [-0.00517439 -0.0245289  -0.00532987], quaternion: (0.6553334238547928, 0.7341898753954419, -0.11819292680017879, 0.13241511431852335)
[INFO] [1778256023.805505793] [pcd_subsriber_node]: Place pose published.
[INFO] [1778256041.085297176] [pcd_subsriber_node]: Inference complete. Shutting down.
```