## grasp pose
```
header:
  stamp:
    sec: 1778252308
    nanosec: 192569608
  frame_id: base_link
pose:
  position:
    x: -0.6763195395469666
    y: -0.028034644201397896
    z: 0.15844085812568665
  orientation:
    x: -0.9854029127068807
    y: -0.09272833278237474
    z: 0.13438703898459678
    w: 0.04819418721328433
```
## placement pose
```
header:
  stamp:
    sec: 1778252308
    nanosec: 197033363
  frame_id: placement_link
pose:
  position:
    x: -0.009639015292550339
    y: 0.014108339454409036
    z: 0.0032944493168866884
  orientation:
    x: 0.67582895599793
    y: 0.7230993463090095
    z: -0.13252932126444117
    w: 0.05308989176310072
```
## vmf python log
```
[INFO] [1778252270.324527577] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778252270.426703825] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778252270.520172698] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778252270.521648168] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778252270, nanosec=417541430), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923497526660141, y=0.13346150176452176, z=0.685765795564265), rotation=geometry_msgs.msg.Quaternion(x=0.6866647341091808, y=0.7248176190291512, z=-0.05593465145173738, w=-0.0015089210227312896)))
[INFO] [1778252270.524234937] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [ 0.00668313 -0.05577078  0.05045695]
Point-cloud center of gravity: [ 0.01497401 -0.02281826  0.05783587]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778252308.188563340] [pcd_subsriber_node]: Inference time: 37.661s
[INFO] [1778252308.192224701] [pcd_subsriber_node]: Predicted translation: [-0.67631954 -0.02803464  0.15844086], quaternion: (-0.9854029127068807, -0.09272833278237474, 0.13438703898459678, 0.04819418721328433)
[INFO] [1778252308.193642186] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-0.08087337  0.99145491 -0.10235578 -0.00963902]
 [ 0.96331103  0.0513824  -0.26342312  0.01410834]
 [-0.25591284 -0.11990436 -0.95923489  0.00329445]
 [ 0.          0.          0.          1.        ]]
[INFO] [1778252308.196776843] [pcd_subsriber_node]: Placement pose translation: [-0.00963902  0.01410834  0.00329445], quaternion: (0.67582895599793, 0.7230993463090095, -0.13252932126444117, 0.05308989176310072)
[INFO] [1778252308.198061299] [pcd_subsriber_node]: Place pose published.
[INFO] [1778252339.341587697] [pcd_subsriber_node]: Inference complete. Shutting down.
```