## grasp pose
```
header:
  stamp:
    sec: 1778677760
    nanosec: 589153746
  frame_id: base_link
pose:
  position:
    x: -0.5681758522987366
    y: 0.16393190622329712
    z: 0.16231726109981537
  orientation:
    x: 0.941975435424822
    y: 0.08555130192068605
    z: -0.3232665725716941
    w: 0.02935944232512293
```
## placement pose
```
header:
  stamp:
    sec: 1778677760
    nanosec: 601693387
  frame_id: placement_link
pose:
  position:
    x: -0.031318416999047194
    y: 0.015256116628423522
    z: 0.0042115490672891
  orientation:
    x: 0.6503095261288021
    y: 0.6868291393409204
    z: -0.22317284069771862
    w: 0.23570561459849354
```
## vmf python log
```
[INFO] [1778677751.568224283] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778677751.670568504] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778677751.772685422] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778677751.774144510] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778677751, nanosec=666678642), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923684476993861, y=0.13345006204849594, z=0.6858196045297265), rotation=geometry_msgs.msg.Quaternion(x=0.6866609086297305, y=0.7248192243182408, z=-0.05595925262758148, w=-0.001565459631910914)))
[INFO] [1778677751.778669523] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 1-4
Bounding Box center: [0.12051831 0.11564205 0.05112053]
Point-cloud center of gravity: [0.12076634 0.14623512 0.06309089]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/1-4_poses.ply
[INFO] [1778677760.583993225] [pcd_subsriber_node]: Inference time: 6.184s
[INFO] [1778677760.588769541] [pcd_subsriber_node]: Predicted translation: [-0.56817585  0.1639319   0.16231726], quaternion: (0.941975435424822, 0.08555130192068605, -0.3232665725716941, 0.02935944232512293)
[INFO] [1778677760.590924300] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-4.30807661e-02  9.98509241e-01  3.35161206e-02 -3.13184170e-02]
 [ 7.88096866e-01  5.45828022e-02 -6.13126417e-01  1.52561166e-02]
 [-6.14041805e-01 -3.84204668e-09 -7.89273500e-01  4.21154907e-03]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778677760.601397377] [pcd_subsriber_node]: Placement pose translation: [-0.03131842  0.01525612  0.00421155], quaternion: (0.6503095261288021, 0.6868291393409204, -0.22317284069771862, 0.23570561459849354)
[INFO] [1778677760.602815216] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/1-4_grasp.ply
[INFO] [1778677768.591880658] [pcd_subsriber_node]: Inference complete. Shutting down.
```