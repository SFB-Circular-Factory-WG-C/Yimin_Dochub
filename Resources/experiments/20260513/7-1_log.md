## vmf python log
```
[INFO] [1778680007.523497574] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778680007.626262110] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778680007.726100392] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778680007.727583936] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778680007, nanosec=595701284), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923448609416738, y=0.13345469172691637, z=0.685821745990794), rotation=geometry_msgs.msg.Quaternion(x=0.686673539454816, y=0.7248079164111055, z=-0.05595134299672107, w=-0.001543287805452652)))
[INFO] [1778680007.732208205] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 7-1
Bounding Box center: [ 0.12878059 -0.17795217  0.0464244 ]
Point-cloud center of gravity: [ 0.16700354 -0.14999932  0.06673995]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
[INFO] [1778680010.507989223] [pcd_subsriber_node]: Inference time: 1.036s
[INFO] [1778680010.508936875] [pcd_subsriber_node]: No prediction returned.
Saved point cloud (no pose) to /home/jetson/wbk_ur10_ws/src/vmf/7-1_no_pose.ply
[INFO] [1778680036.275494868] [pcd_subsriber_node]: Inference complete. Shutting down.
```