## grasp pose
```
header:
  stamp:
    sec: 1778681575
    nanosec: 130900189
  frame_id: base_link
pose:
  position:
    x: -0.80028235912323
    y: -0.1615113615989685
    z: 0.14428925514221191
  orientation:
    x: 0.9879624492372752
    y: 0.014608846222431748
    z: -0.15398570047325366
    w: 0.002276962646874949
```
## placement pose
```
header:
  stamp:
    sec: 1778681575
    nanosec: 140760807
  frame_id: placement_link
pose:
  position:
    x: -0.03454804712523496
    y: 0.014243555346205028
    z: -0.019354736028629915
  orientation:
    x: 0.7023291643075376
    y: 0.694994219597376
    z: -0.10946635307631022
    w: 0.10832311495467288
```
## vmf python log
```
[INFO] [1778681565.889951641] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778681565.992137610] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778681566.092458759] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778681566.093945608] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778681565, nanosec=950843061), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923757776082252, y=0.13343600638912162, z=0.6858126308492274), rotation=geometry_msgs.msg.Quaternion(x=0.6866663034003787, y=0.72481450020073, z=-0.05595457336075723, w=-0.0015536387098541804)))
[INFO] [1778681566.098510702] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 9-1
Bounding Box center: [-0.11695186 -0.19719857  0.05332901]
Point-cloud center of gravity: [-0.10930917 -0.19600596  0.07195897]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/9-1_poses.ply
[INFO] [1778681575.123701415] [pcd_subsriber_node]: Inference time: 6.048s
[INFO] [1778681575.130250977] [pcd_subsriber_node]: Predicted translation: [-0.80028236 -0.16151136  0.14428926], quaternion: (0.9879624492372752, 0.014608846222431748, -0.15398570047325366, 0.002276962646874949)
[INFO] [1778681575.137366580] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[ 1.00003047e-02  9.99944897e-01 -3.19494765e-03 -3.45480471e-02]
 [ 9.52513960e-01 -1.04982748e-02 -3.04313953e-01  1.42435553e-02]
 [-3.04330707e-01  2.82781798e-10 -9.52566445e-01 -1.93547360e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778681575.140521660] [pcd_subsriber_node]: Placement pose translation: [-0.03454805  0.01424356 -0.01935474], quaternion: (0.7023291643075376, 0.694994219597376, -0.10946635307631022, 0.10832311495467288)
[INFO] [1778681575.145907780] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/9-1_grasp.ply
[INFO] [1778681577.635707511] [pcd_subsriber_node]: Inference complete. Shutting down.
```