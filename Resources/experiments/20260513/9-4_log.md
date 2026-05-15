## grasp pose
```
header:
  stamp:
    sec: 1778682299
    nanosec: 473943329
  frame_id: base_link
pose:
  position:
    x: -0.8234191536903381
    y: -0.13968241214752197
    z: 0.1455962359905243
  orientation:
    x: 0.98952934826887
    y: 0.07037964268634236
    z: -0.12569190732654673
    w: 0.008939756291037448
```
## placement pose
```
header:
  stamp:
    sec: 1778682299
    nanosec: 487247191
  frame_id: placement_link
pose:
  position:
    x: -0.04375938892105315
    y: 0.010112388131490846
    z: -0.018535171763726765
  orientation:
    x: 0.7048312594654371
    y: 0.6980934905259802
    z: -0.08952901103235274
    w: 0.0886731658913086
```
## vmf python log
```
[INFO] [1778682282.311964379] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778682282.414152814] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778682282.514056182] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778682282.515501926] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778682282, nanosec=392917804), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923802601028104, y=0.1334345445407399, z=0.6857831073436859), rotation=geometry_msgs.msg.Quaternion(x=0.6866519768797147, y=0.7248296321945832, z=-0.05593531618356611, w=-0.001518994170289923)))
[INFO] [1778682282.520095613] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 9-4
Bounding Box center: [-0.12860937 -0.18218234  0.0539408 ]
Point-cloud center of gravity: [-0.12500334 -0.18675193  0.07232202]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/9-4_poses.ply
[INFO] [1778682299.467912346] [pcd_subsriber_node]: Inference time: 5.784s
[INFO] [1778682299.473544788] [pcd_subsriber_node]: Predicted translation: [-0.82341915 -0.13968241  0.14559624], quaternion: (0.98952934826887, 0.07037964268634236, -0.12569190732654673, 0.008939756291037448)
[INFO] [1778682299.475964867] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[ 9.30006943e-03  9.99953908e-01 -2.40137181e-03 -4.37593889e-02]
 [ 9.68198597e-01 -9.60509815e-03 -2.49998486e-01  1.01123881e-02]
 [-2.50010014e-01 -8.46394510e-10 -9.68243301e-01 -1.85351718e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778682299.486691333] [pcd_subsriber_node]: Placement pose translation: [-0.04375939  0.01011239 -0.01853517], quaternion: (0.7048312594654371, 0.6980934905259802, -0.08952901103235274, 0.0886731658913086)
[INFO] [1778682299.488461151] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/9-4_grasp.ply
[INFO] [1778682301.636295444] [pcd_subsriber_node]: Inference complete. Shutting down.
```