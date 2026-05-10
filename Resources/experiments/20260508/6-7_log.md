## grasp pose
```
header:
  stamp:
    sec: 1778256372
    nanosec: 787128493
  frame_id: base_link
pose:
  position:
    x: -0.8249494433403015
    y: 0.00023020186927169561
    z: 0.15339283645153046
  orientation:
    x: -0.989773353876967
    y: -0.00404509342877582
    z: 0.12933063704310135
    w: 0.06004940879241502
```
## placement pose
```
header:
  stamp:
    sec: 1778256372
    nanosec: 802724786
  frame_id: placement_link
pose:
  position:
    x: -0.035653717821300696
    y: -0.02303841550061214
    z: -0.00018007677342968265
  orientation:
    x: 0.6328015779407876
    y: 0.7610714924580039
    z: -0.12910770559548523
    w: 0.06052723915119012
```
## vmf python log
```
[INFO] [1778256355.653435821] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778256355.755871267] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778256355.850762639] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778256355.852287615] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778256355, nanosec=712349374), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923745735881031, y=0.13345795351039866, z=0.685784098790299), rotation=geometry_msgs.msg.Quaternion(x=0.6866662483315473, y=0.7248167175770821, z=-0.055926986883425574, w=-0.0015367046572302276)))
[INFO] [1778256355.854797837] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [-0.10445391 -0.05524941  0.0480719 ]
Point-cloud center of gravity: [-0.11251508 -0.02254264  0.05707392]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778256372.781598432] [pcd_subsriber_node]: Inference time: 16.924s
[INFO] [1778256372.786802082] [pcd_subsriber_node]: Predicted translation: [-8.2494944e-01  2.3020187e-04  1.5339284e-01], quaternion: (-0.989773353876967, -0.00404509342877582, 0.12933063704310135, 0.06004940879241502)
[INFO] [1778256372.788264496] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-1.91797237e-01  9.78843610e-01 -7.12680106e-02 -3.56537178e-02]
 [ 9.47585439e-01  1.65786736e-01 -2.73123853e-01 -2.30384155e-02]
 [-2.55530238e-01 -1.19916931e-01 -9.59335327e-01 -1.80076773e-04]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778256372.802329414] [pcd_subsriber_node]: Placement pose translation: [-0.03565372 -0.02303842 -0.00018008], quaternion: (0.6328015779407876, 0.7610714924580039, -0.12910770559548523, 0.06052723915119012)
[INFO] [1778256372.807748847] [pcd_subsriber_node]: Place pose published.
[INFO] [1778256388.241193278] [pcd_subsriber_node]: Inference complete. Shutting down.
```