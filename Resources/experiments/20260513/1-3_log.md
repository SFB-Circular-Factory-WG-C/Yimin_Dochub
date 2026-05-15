## grasp pose
```
header:
  stamp:
    sec: 1778677320
    nanosec: 455230551
  frame_id: base_link
pose:
  position:
    x: -0.5337145328521729
    y: 0.15902256965637207
    z: 0.15889684855937958
  orientation:
    x: -0.931319889475656
    y: -0.055317568240914605
    z: 0.35885452407596385
    w: 0.02840177214852646
```
## placement pose
```
header:
  stamp:
    sec: 1778677320
    nanosec: 466520547
  frame_id: placement_link
pose:
  position:
    x: -0.03496898026090922
    y: 0.0334017349268485
    z: 0.0034639977934729504
  orientation:
    x: 0.7028950020257746
    y: 0.6134781043713013
    z: -0.30125152091442714
    w: 0.19705520224273612
```
## vmf python log
```
[INFO] [1778677295.078774893] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778677295.181105360] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778677295.287511720] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778677295.288984199] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778677295, nanosec=184915979), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923607431799934, y=0.1334627024165498, z=0.685824364895934), rotation=geometry_msgs.msg.Quaternion(x=0.686673335680419, y=0.7248086706946506, z=-0.05594380490416846, w=-0.0015529469485045023)))
[INFO] [1778677295.293552647] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 1-3
Bounding Box center: [0.14232282 0.10493491 0.04676422]
Point-cloud center of gravity: [0.14318718 0.1286159  0.06210148]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/1-3_poses.ply
[INFO] [1778677320.446411390] [pcd_subsriber_node]: Inference time: 13.519s
[INFO] [1778677320.454881936] [pcd_subsriber_node]: Predicted translation: [-0.53371453  0.15902257  0.15889685], quaternion: (-0.931319889475656, -0.055317568240914605, 0.35885452407596385, 0.02840177214852646)
[INFO] [1778677320.457031069] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[ 0.06578428  0.98114779 -0.18171827 -0.03496898]
 [ 0.74369505 -0.16962773 -0.64664062  0.03340173]
 [-0.6652745  -0.0926042  -0.74083358  0.003464  ]
 [ 0.          0.          0.          1.        ]]
[INFO] [1778677320.466224125] [pcd_subsriber_node]: Placement pose translation: [-0.03496898  0.03340173  0.003464  ], quaternion: (0.7028950020257746, 0.6134781043713013, -0.30125152091442714, 0.19705520224273612)
[INFO] [1778677320.467719804] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/1-3_grasp.ply
[INFO] [1778677323.584928924] [pcd_subsriber_node]: Inference complete. Shutting down.
```