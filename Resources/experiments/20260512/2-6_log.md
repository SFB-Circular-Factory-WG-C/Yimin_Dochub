## grasp pose
```
header:
  stamp:
    sec: 1778602789
    nanosec: 392511220
  frame_id: base_link
pose:
  position:
    x: -0.6575179100036621
    y: 0.15288378298282623
    z: 0.16418103873729706
  orientation:
    x: -0.9910831375298647
    y: -0.038626274721115184
    z: 0.11429595384297098
    w: 0.05655670022423531
```
## placement pose
```
header:
  stamp:
    sec: 1778602789
    nanosec: 401894077
  frame_id: placement_link
pose:
  position:
    x: -0.01863550940026948
    y: 0.01244715853503453
    z: 0.004916643981878743
  orientation:
    x: 0.6946759525274384
    y: 0.7079287365164354
    z: -0.12196296267174205
    w: 0.03724594918472157
```
## vmf python log
```
[INFO] [1778602776.219412718] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778602776.321594675] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778602776.422907901] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778602776.424350763] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778602776, nanosec=306422832), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923363932437656, y=0.1334819179819615, z=0.6858244248999831), rotation=geometry_msgs.msg.Quaternion(x=0.6866976884254945, y=0.7247853816961528, z=-0.055947000772480215, w=-0.0015389266417533203)))
[INFO] [1778602776.428895259] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 2-6
Bounding Box center: [0.0294644  0.11875311 0.05382916]
Point-cloud center of gravity: [0.03284693 0.14834012 0.06269963]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/2-6_poses.ply
[INFO] [1778602789.388104488] [pcd_subsriber_node]: Inference time: 8.855s
[INFO] [1778602789.392142248] [pcd_subsriber_node]: Predicted translation: [-0.6575179   0.15288378  0.16418104], quaternion: (-0.9910831375298647, -0.038626274721115184, 0.11429595384297098, 0.05655670022423531)
[INFO] [1778602789.393715130] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-0.03207612  0.99264746 -0.11671453 -0.01863551]
 [ 0.97447691  0.00510071 -0.2244299   0.01244716]
 [-0.22218443 -0.12093445 -0.96747559  0.00491664]
 [ 0.          0.          0.          1.        ]]
[INFO] [1778602789.401588884] [pcd_subsriber_node]: Placement pose translation: [-0.01863551  0.01244716  0.00491664], quaternion: (0.6946759525274384, 0.7079287365164354, -0.12196296267174205, 0.03724594918472157)
[INFO] [1778602789.402897437] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/2-6_grasp.ply
[INFO] [1778602793.962526909] [pcd_subsriber_node]: Inference complete. Shutting down.
```