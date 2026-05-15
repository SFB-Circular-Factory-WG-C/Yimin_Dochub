## vmf python log
```
[INFO] [1778679649.414798711] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778679649.516814028] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778679649.618377099] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778679649.619854449] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778679649, nanosec=526883375), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923481629106445, y=0.13344573516926345, z=0.6858104705639576), rotation=geometry_msgs.msg.Quaternion(x=0.686661703587176, y=0.724818852904649, z=-0.055954835238522745, w=-0.0015465167880915856)))
[INFO] [1778679649.624420684] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 4-4
Bounding Box center: [ 0.13214932 -0.04584369  0.05783157]
Point-cloud center of gravity: [ 0.12230619 -0.03465141  0.07613181]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
[INFO] [1778679653.708374950] [pcd_subsriber_node]: Inference time: 1.027s
[INFO] [1778679653.709390615] [pcd_subsriber_node]: No prediction returned.
Saved point cloud (no pose) to /home/jetson/wbk_ur10_ws/src/vmf/4-4_no_pose.ply
[INFO] [1778679657.059751663] [pcd_subsriber_node]: Inference complete. Shutting down.
```