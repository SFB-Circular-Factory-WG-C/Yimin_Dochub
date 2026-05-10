## grasp pose
```
header:
  stamp:
    sec: 1778253403
    nanosec: 542589015
  frame_id: base_link
pose:
  position:
    x: -0.6522355079650879
    y: -0.03645462542772293
    z: 0.1583133339881897
  orientation:
    x: -0.9914936750421227
    y: 0.0004149575677258599
    z: 0.1150016607151947
    w: 0.06094865211331499
```
## placement pose
```
header:
  stamp:
    sec: 1778253403
    nanosec: 548073393
  frame_id: placement_link
pose:
  position:
    x: -0.01439902109374916
    y: 0.009783369525164565
    z: 0.002016685294757792
  orientation:
    x: 0.7173917999837433
    y: 0.6844040357227305
    z: -0.12526564371228374
    w: 0.035336096387007264
```
## vmf python log
```
[INFO] [1778253384.323928350] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778253384.426068171] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778253384.526046543] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778253384.527570882] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778253384, nanosec=412524361), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923711510689259, y=0.1334275941424663, z=0.6858110626854707), rotation=geometry_msgs.msg.Quaternion(x=0.6866524545545758, y=0.7248279926672947, z=-0.05594985365767811, w=-0.0015497018296797168)))
[INFO] [1778253384.530140250] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [ 0.03685773 -0.06622606  0.05179998]
Point-cloud center of gravity: [ 0.04045596 -0.03635452  0.05879332]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778253403.535317569] [pcd_subsriber_node]: Inference time: 19.003s
[INFO] [1778253403.542230795] [pcd_subsriber_node]: Predicted translation: [-0.6522355  -0.03645463  0.15831333], quaternion: (-0.9914936750421227, 0.0004149575677258599, 0.1150016607151947, 0.06094865211331499)
[INFO] [1778253403.543923908] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[ 0.03179927  0.99082453 -0.13136075 -0.01439902]
 [ 0.97311895 -0.06068495 -0.22216428  0.00978337]
 [-0.22809744 -0.12076498 -0.96611977  0.00201669]
 [ 0.          0.          0.          1.        ]]
[INFO] [1778253403.547813864] [pcd_subsriber_node]: Placement pose translation: [-0.01439902  0.00978337  0.00201669], quaternion: (0.7173917999837433, 0.6844040357227305, -0.12526564371228374, 0.035336096387007264)
[INFO] [1778253403.549051090] [pcd_subsriber_node]: Place pose published.
[INFO] [1778253459.657534305] [pcd_subsriber_node]: Inference complete. Shutting down.
```