## grasp pose
```
header:
  stamp:
    sec: 1778683049
    nanosec: 852581896
  frame_id: base_link
pose:
  position:
    x: -0.8090125322341919
    y: -0.1827513575553894
    z: 0.14678014814853668
  orientation:
    x: -0.9949573640857289
    y: 0.015760446904535458
    z: 0.09904034901318327
    w: 0.001568831506111931
```
## placement pose
```
header:
  stamp:
    sec: 1778683049
    nanosec: 861798240
  frame_id: placement_link
pose:
  position:
    x: -0.013089881346567911
    y: 0.014446573362232096
    z: -0.014529325214159255
  orientation:
    x: 0.7201734349623926
    y: 0.6866868066929037
    z: -0.07168772770670043
    w: 0.06835439111440379
```
## vmf python log
```
[INFO] [1778683038.429540731] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778683038.532236276] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778683038.629407589] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778683038.630838737] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778683038, nanosec=508826206), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923517414087649, y=0.1334665180080358, z=0.685807002364031), rotation=geometry_msgs.msg.Quaternion(x=0.6866872811864466, y=0.7247957668435352, z=-0.0559397862550802, w=-0.0015538777924103818)))
[INFO] [1778683038.635452861] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 9-5
Bounding Box center: [-0.12728277 -0.19697706  0.04901627]
Point-cloud center of gravity: [-0.11921507 -0.195162    0.07160267]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/9-5_poses.ply
[INFO] [1778683049.848036157] [pcd_subsriber_node]: Inference time: 4.123s
[INFO] [1778683049.852243293] [pcd_subsriber_node]: Predicted translation: [-0.80901253 -0.18275136  0.14678015], quaternion: (-0.9949573640857289, 0.015760446904535458, 0.09904034901318327, 0.001568831506111931)
[INFO] [1778683049.853716266] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[ 4.66442007e-02  9.98867608e-01 -9.37907732e-03 -1.30898813e-02]
 [ 9.79266898e-01 -4.75778181e-02 -1.96908079e-01  1.44465734e-02]
 [-1.97131321e-01 -1.08985820e-10 -9.80377138e-01 -1.45293252e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778683049.861552729] [pcd_subsriber_node]: Placement pose translation: [-0.01308988  0.01444657 -0.01452933], quaternion: (0.7201734349623926, 0.6866868066929037, -0.07168772770670043, 0.06835439111440379)
[INFO] [1778683049.866613619] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/9-5_grasp.ply
[INFO] [1778683051.608570950] [pcd_subsriber_node]: Inference complete. Shutting down.
```