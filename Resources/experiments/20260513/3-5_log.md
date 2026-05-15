## grasp pose
```
header:
  stamp:
    sec: 1778678881
    nanosec: 197402571
  frame_id: base_link
pose:
  position:
    x: -0.8195549249649048
    y: 0.16364383697509766
    z: 0.15856337547302246
  orientation:
    x: 0.9935787212675531
    y: 0.06354978601965897
    z: -0.09341866772216238
    w: 0.005975103530984594
```
## placement pose
```
header:
  stamp:
    sec: 1778678881
    nanosec: 206996563
  frame_id: placement_link
pose:
  position:
    x: -0.04622319779639228
    y: 0.009429861048325439
    z: -0.010899885921914515
  orientation:
    x: 0.6531653845862141
    y: 0.7514068339933057
    z: -0.06141218795625767
    w: 0.07064908617190309
```
## vmf python log
```
[INFO] [1778678875.947965380] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778678876.050259247] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778678876.154847185] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778678876.156326325] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778678876, nanosec=58728063), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923691582348869, y=0.1334561226899599, z=0.6858258625972603), rotation=geometry_msgs.msg.Quaternion(x=0.6866793693639307, y=0.7248007898734564, z=-0.05597209538251188, w=-0.0015437704684139759)))
[INFO] [1778678876.160866947] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 Bounding Box center: [-0.129936    0.11794355  0.05975156]
Point-cloud center of gravity: [-0.12913358  0.11712898  0.07717496]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/vmf_20260513_132756_poses.ply
[INFO] [1778678881.191345657] [pcd_subsriber_node]: Inference time: 5.026s
[INFO] [1778678881.197061923] [pcd_subsriber_node]: Predicted translation: [-0.8195549   0.16364384  0.15856338], quaternion: (0.9935787212675531, 0.06354978601965897, -0.09341866772216238, 0.005975103530984594)
[INFO] [1778678881.198637193] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-1.36767376e-01  9.90263300e-01  2.59477828e-02 -4.62231978e-02]
 [ 9.72908452e-01  1.39207049e-01 -1.84582159e-01  9.42986105e-03]
 [-1.86397046e-01 -6.60664190e-10 -9.82474506e-01 -1.08998859e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778678881.206716300] [pcd_subsriber_node]: Placement pose translation: [-0.0462232   0.00942986 -0.01089989], quaternion: (0.6531653845862141, 0.7514068339933057, -0.06141218795625767, 0.07064908617190309)
[INFO] [1778678881.208321395] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/vmf_20260513_132756_grasp.ply
[INFO] [1778678886.359555707] [pcd_subsriber_node]: Inference complete. Shutting down.
```