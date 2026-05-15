## vmf python log
```
[INFO] [1778680104.453592774] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778680104.555697138] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778680104.656937280] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778680104.658407828] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778680104, nanosec=516684251), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923593823425289, y=0.13343725616025348, z=0.6858267301164265), rotation=geometry_msgs.msg.Quaternion(x=0.6866536495593202, y=0.724826449238127, z=-0.05595510151387992, w=-0.0015526256105999495)))
[INFO] [1778680104.662995481] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 7-2
Bounding Box center: [ 0.00434046 -0.16715382  0.06261567]
Point-cloud center of gravity: [ 0.12023089 -0.1640109   0.06825919]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
[INFO] [1778680108.222724323] [pcd_subsriber_node]: Inference time: 1.155s
[INFO] [1778680108.223821226] [pcd_subsriber_node]: No prediction returned.
Saved point cloud (no pose) to /home/jetson/wbk_ur10_ws/src/vmf/7-2_no_pose.ply
[INFO] [1778680133.455154233] [pcd_subsriber_node]: Inference complete. Shutting down.
```