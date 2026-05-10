## grasp pose
```
header:
  stamp:
    sec: 1778254832
    nanosec: 425595199
  frame_id: base_link
pose:
  position:
    x: -0.7955082058906555
    y: 0.014311565086245537
    z: 0.15500497817993164
  orientation:
    x: -0.9835742063093145
    y: -0.002597700630311728
    z: 0.1705858590831048
    w: 0.058953348635307545
```
## placement pose
```
header:
  stamp:
    sec: 1778254832
    nanosec: 431725660
  frame_id: placement_link
pose:
  position:
    x: -0.025124370994889983
    y: -0.0038003646032298555
    z: 3.8256141713932657e-05
  orientation:
    x: 0.6799827655173709
    y: 0.7106675774773012
    z: -0.1607452480957036
    w: 0.08207312676231382
```
## vmf python log
```
[INFO] [1778254814.285802493] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778254814.388782754] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778254814.484219167] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778254814.485714029] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778254814, nanosec=362544231), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923807469314551, y=0.13343347041174067, z=0.6858496559251873), rotation=geometry_msgs.msg.Quaternion(x=0.6866473863443854, y=0.7248318941028298, z=-0.0559607770013847, w=-0.0015759299254012515)))
[INFO] [1778254814.488237659] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [-0.08856633 -0.02746428  0.04948302]
Point-cloud center of gravity: [-0.0968063   0.00558142  0.05845042]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778254832.419570118] [pcd_subsriber_node]: Inference time: 17.929s
[INFO] [1778254832.425280373] [pcd_subsriber_node]: Predicted translation: [-0.7955082   0.01431157  0.15500498], quaternion: (-0.9835742063093145, -0.002597700630311728, 0.1705858590831048, 0.058953348635307545)
[INFO] [1778254832.426911976] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-6.17748841e-02  9.92869126e-01 -1.01954583e-01 -2.51243710e-02]
 [ 9.40097728e-01  2.35688077e-02 -3.40089497e-01 -3.80036460e-03]
 [-3.35261434e-01 -1.16856247e-01 -9.34849977e-01  3.82561417e-05]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778254832.431458036] [pcd_subsriber_node]: Placement pose translation: [-2.51243710e-02 -3.80036460e-03  3.82561417e-05], quaternion: (0.6799827655173709, 0.7106675774773012, -0.1607452480957036, 0.08207312676231382)
[INFO] [1778254832.432819550] [pcd_subsriber_node]: Place pose published.
[INFO] [1778254852.793003079] [pcd_subsriber_node]: Inference complete. Shutting down.
```