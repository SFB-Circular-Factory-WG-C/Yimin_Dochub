## grasp pose
```
header:
  stamp:
    sec: 1778602530
    nanosec: 442011169
  frame_id: base_link
pose:
  position:
    x: -0.6614124178886414
    y: 0.13645339012145996
    z: 0.16196419298648834
  orientation:
    x: 0.9839330600552944
    y: 0.052947146302663645
    z: -0.17025977575920792
    w: 0.009161975062889191
```
## placement pose
```
header:
  stamp:
    sec: 1778602530
    nanosec: 448144611
  frame_id: placement_link
pose:
  position:
    x: -0.007318159085568765
    y: 0.015565398728555335
    z: 0.0026803947307423304
  orientation:
    x: 0.673877453390682
    y: 0.7188997432043144
    z: -0.1166077620869006
    w: 0.12439841981932394
```
## vmf python log
```
[INFO] [1778602522.035616709] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778602522.137566746] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778602522.240762166] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778602522.242220132] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778602522, nanosec=128343525), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923487724852679, y=0.13349965570565683, z=0.6858563303156647), rotation=geometry_msgs.msg.Quaternion(x=0.6866916681931314, y=0.7247895744379561, z=-0.05596514769626058, w=-0.001589925632370709)))
[INFO] [1778602522.246812981] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 2-5
Bounding Box center: [0.02117254 0.11229865 0.05366503]
Point-cloud center of gravity: [0.02552804 0.14465044 0.06290257]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/2-5_poses.ply
[INFO] [1778602530.434761660] [pcd_subsriber_node]: Inference time: 4.838s
[INFO] [1778602530.441604724] [pcd_subsriber_node]: Predicted translation: [-0.6614124   0.13645339  0.1619642 ], quaternion: (0.9839330600552944, 0.052947146302663645, -0.17025977575920792, 0.009161975062889191)
[INFO] [1778602530.443882492] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-6.08284244e-02  9.97912367e-01  2.17013003e-02 -7.31815909e-03]
 [ 9.39889053e-01  6.45836202e-02 -3.35317177e-01  1.55653987e-02]
 [-3.36018682e-01  6.40922870e-11 -9.41855371e-01  2.68039473e-03]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778602530.447901435] [pcd_subsriber_node]: Placement pose translation: [-0.00731816  0.0155654   0.00268039], quaternion: (0.673877453390682, 0.7188997432043144, -0.1166077620869006, 0.12439841981932394)
[INFO] [1778602530.449204965] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/2-5_grasp.ply
[INFO] [1778602535.399052728] [pcd_subsriber_node]: Inference complete. Shutting down.
```