## vmf python log
```
8-2
[INFO] [1778680672.473280193] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778680672.575440049] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778680672.679192560] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778680672.680644188] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778680672, nanosec=547020069), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923501348913513, y=0.13346018331582649, z=0.6858421332979178), rotation=geometry_msgs.msg.Quaternion(x=0.6866706648564732, y=0.7248098685807977, z=-0.055960682020471074, w=-0.0015666847990053825)))
[INFO] [1778680672.685135780] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 8-2
Bounding Box center: [ 0.03603428 -0.19366906  0.06212028]
Point-cloud center of gravity: [ 0.03874904 -0.1984219   0.07369947]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
[INFO] [1778680679.033675943] [pcd_subsriber_node]: Inference time: 0.979s
[INFO] [1778680679.034673733] [pcd_subsriber_node]: No prediction returned.
Saved point cloud (no pose) to /home/jetson/wbk_ur10_ws/src/vmf/8-2_no_pose.ply
[INFO] [1778680688.331948170] [pcd_subsriber_node]: Inference complete. Shutting down.
```