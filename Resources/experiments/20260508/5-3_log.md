## grasp pose
```
header:
  stamp:
    sec: 1778251657
    nanosec: 466948118
  frame_id: base_link
pose:
  position:
    x: -0.6905361413955688
    y: -0.015778347849845886
    z: 0.16412559151649475
  orientation:
    x: -0.9920740065689639
    y: -0.06750832354308492
    z: 0.09032409957308861
    w: 0.05543779197367597
```
## placement pose
```
header:
  stamp:
    sec: 1778251657
    nanosec: 493614726
  frame_id: placement_link
pose:
  position:
    x: -0.03111650476727143
    y: 0.008472252582532369
    z: 0.005627198208549344
  orientation:
    x: 0.681039774605741
    y: 0.7245364269862011
    z: -0.10391134464777872
    w: 0.02083803801995511
```
## vmf python log
```
[INFO] [1778251592.395422947] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778251592.498064390] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778251592.591154512] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778251592.592666823] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778251592, nanosec=472255324), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923721023328399, y=0.13349892414854783, z=0.6857705299164154), rotation=geometry_msgs.msg.Quaternion(x=0.6867057978972332, y=0.724779818872691, z=-0.055919545294879816, w=-0.0015380972684299655)))
[INFO] [1778251592.595162337] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [ 0.00188664 -0.06350787  0.05510638]
Point-cloud center of gravity: [ 0.00474487 -0.03136444  0.0598904 ]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778251657.461802208] [pcd_subsriber_node]: Inference time: 64.864s
[INFO] [1778251657.466558120] [pcd_subsriber_node]: Predicted translation: [-0.69053614 -0.01577835  0.16412559], quaternion: (-0.9920740065689639, -0.06750832354308492, 0.09032409957308861, 0.05543779197367597)
[INFO] [1778251657.468836985] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-0.0715012   0.9912069  -0.11133968 -0.0311165 ]
 [ 0.98254563  0.05077452 -0.17895819  0.00847225]
 [-0.17173135 -0.12219205 -0.97753644  0.0056272 ]
 [ 0.          0.          0.          1.        ]]
[INFO] [1778251657.493306523] [pcd_subsriber_node]: Placement pose translation: [-0.0311165   0.00847225  0.0056272 ], quaternion: (0.681039774605741, 0.7245364269862011, -0.10391134464777872, 0.02083803801995511)
[INFO] [1778251657.495072313] [pcd_subsriber_node]: Place pose published.
[INFO] [1778251694.924931262] [pcd_subsriber_node]: Inference complete. Shutting down.
```