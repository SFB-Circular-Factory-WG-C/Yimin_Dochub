## grasp pose
```
header:
  stamp:
    sec: 1778681842
    nanosec: 136285962
  frame_id: base_link
pose:
  position:
    x: -0.8000267744064331
    y: -0.14633715152740479
    z: 0.1460600197315216
  orientation:
    x: -0.99248937954148
    y: 0.05357019016829302
    z: 0.10981771831061088
    w: 0.005927475664421472
```
## placement pose
```
header:
  stamp:
    sec: 1778681842
    nanosec: 148094791
  frame_id: placement_link
pose:
  position:
    x: -0.03858946541901767
    y: 0.010611945482674856
    z: -0.018104255913099165
  orientation:
    x: 0.7111996110155917
    y: 0.6943342477890437
    z: -0.07869335297049597
    w: 0.07682722067571661
```
## vmf python log
```
[INFO] [1778681831.957374985] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778681832.060330751] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778681832.167886595] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778681832.169378459] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778681832, nanosec=48704020), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.692384215989649, y=0.13344238560525365, z=0.6857437229633919), rotation=geometry_msgs.msg.Quaternion(x=0.6866574782965478, y=0.7248275428802592, z=-0.055894773911596994, w=-0.0015214568555703047)))
[INFO] [1778681832.173791714] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 9-2
Bounding Box center: [-0.11368905 -0.18346547  0.05409242]
Point-cloud center of gravity: [-0.11397763 -0.18433909  0.07223614]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/9-2_poses.ply
[INFO] [1778681842.130667895] [pcd_subsriber_node]: Inference time: 5.725s
[INFO] [1778681842.135961374] [pcd_subsriber_node]: Predicted translation: [-0.8000268  -0.14633715  0.14606002], quaternion: (-0.99248937954148, 0.05357019016829302, 0.10981771831061088, 0.005927475664421472)
[INFO] [1778681842.137348946] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[ 2.34146178e-02  9.99712154e-01 -5.24582256e-03 -3.85894654e-02]
 [ 9.75528939e-01 -2.39950671e-02 -2.18557964e-01  1.06119455e-02]
 [-2.18620911e-01 -2.07557860e-10 -9.75809932e-01 -1.81042559e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778681842.147794396] [pcd_subsriber_node]: Placement pose translation: [-0.03858947  0.01061195 -0.01810426], quaternion: (0.7111996110155917, 0.6943342477890437, -0.07869335297049597, 0.07682722067571661)
[INFO] [1778681842.149054411] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/9-2_grasp.ply
[INFO] [1778681845.760091244] [pcd_subsriber_node]: Inference complete. Shutting down.
```