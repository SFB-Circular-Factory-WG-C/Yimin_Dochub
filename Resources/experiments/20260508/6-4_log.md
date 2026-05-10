## grasp pose
```
header:
  stamp:
    sec: 1778255185
    nanosec: 418432025
  frame_id: base_link
pose:
  position:
    x: -0.7823805212974548
    y: 0.04407961666584015
    z: 0.15842707455158234
  orientation:
    x: -0.9867086717722624
    y: 0.027830557863244954
    z: 0.14664863950964357
    w: 0.06423109549438613
```
## placement pose
```
header:
  stamp:
    sec: 1778255185
    nanosec: 422852201
  frame_id: placement_link
pose:
  position:
    x: -0.031207859552669456
    y: -0.0017552316431314052
    z: 0.003206680450166116
  orientation:
    x: 0.7251884770293692
    y: 0.6696791917907223
    z: -0.14977912307368563
    w: 0.05654791912729268
```
## vmf python log
```
[INFO] [1778255165.430514836] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778255165.532740617] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778255165.630273540] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778255165.631778357] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778255165, nanosec=496507412), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923561172066686, y=0.1334497411899062, z=0.6858035133985186), rotation=geometry_msgs.msg.Quaternion(x=0.6866594297637609, y=0.7248224650626938, z=-0.05593633010202633, w=-0.0015325246046891832)))
[INFO] [1778255165.634342313] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [-0.07341851 -0.00102824  0.0510973 ]
Point-cloud center of gravity: [-0.08638804  0.02686975  0.05734349]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778255185.410700284] [pcd_subsriber_node]: Inference time: 19.774s
[INFO] [1778255185.418118575] [pcd_subsriber_node]: Predicted translation: [-0.7823805   0.04407962  0.15842707], quaternion: (-0.9867086717722624, 0.027830557863244954, 0.14664863950964357, 0.06423109549438613)
[INFO] [1778255185.419534461] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[ 0.05819199  0.98822669 -0.14149826 -0.03120786]
 [ 0.95434786 -0.09666423 -0.28262375 -0.00175523]
 [-0.29297411 -0.11859213 -0.94873708  0.00320668]
 [ 0.          0.          0.          1.        ]]
[INFO] [1778255185.422628866] [pcd_subsriber_node]: Placement pose translation: [-0.03120786 -0.00175523  0.00320668], quaternion: (0.7251884770293692, 0.6696791917907223, -0.14977912307368563, 0.05654791912729268)
[INFO] [1778255185.424030928] [pcd_subsriber_node]: Place pose published.
[INFO] [1778255202.565835661] [pcd_subsriber_node]: Inference complete. Shutting down.
```