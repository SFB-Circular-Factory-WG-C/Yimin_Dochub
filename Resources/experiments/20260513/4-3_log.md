## vmf python log
```
[INFO] [1778679613.413630633] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778679613.515698585] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778679613.616066036] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778679613.617526620] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778679613, nanosec=530703555), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923528221943459, y=0.13345523489363012, z=0.6858084488051263), rotation=geometry_msgs.msg.Quaternion(x=0.6866656119949417, y=0.7248152389487101, z=-0.055953900268469456, w=-0.001538744970005154)))
[INFO] [1778679613.622024666] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 4-4
Bounding Box center: [ 0.16364236 -0.04810553  0.06577228]
Point-cloud center of gravity: [ 0.15953884 -0.0382085   0.07596596]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
[INFO] [1778679618.194368178] [pcd_subsriber_node]: Inference time: 1.061s
[INFO] [1778679618.195300031] [pcd_subsriber_node]: No prediction returned.
Saved point cloud (no pose) to /home/jetson/wbk_ur10_ws/src/vmf/4-4_no_pose.ply
[INFO] [1778679620.726734511] [pcd_subsriber_node]: Inference complete. Shutting down.
```