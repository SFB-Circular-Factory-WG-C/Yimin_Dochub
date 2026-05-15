## grasp pose
```
header:
  stamp:
    sec: 1778681017
    nanosec: 659377109
  frame_id: base_link
pose:
  position:
    x: -0.6615482568740845
    y: -0.17266973853111267
    z: 0.1478680819272995
  orientation:
    x: -0.9793963164618764
    y: 0.008785210428868484
    z: 0.2017483591233684
    w: 0.0018096877237298594
```
## placement pose
```
header:
  stamp:
    sec: 1778681017
    nanosec: 666116663
  frame_id: placement_link
pose:
  position:
    x: -0.019937222870034826
    y: 0.04333370791556779
    z: -0.01282056902773665
  orientation:
    x: 0.6816580923955353
    y: 0.7033040389911775
    z: -0.140416495764379
    w: 0.14487540000851926
```
## vmf python log
```
[INFO] [1778680995.877064215] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778680995.979019548] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778680996.078661438] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778680996.080149353] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778680995, nanosec=938810083), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923377053753526, y=0.13347003436441396, z=0.6858018266017127), rotation=geometry_msgs.msg.Quaternion(x=0.6866742855765435, y=0.724809471843647, z=-0.05592208931895213, w=-0.0015410969730287915)))
[INFO] [1778680996.084732654] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 8-4
Bounding Box center: [-0.03907661 -0.18595603  0.04577933]
Point-cloud center of gravity: [ 0.02745668 -0.19494772  0.07359797]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/8-4_poses.ply
[INFO] [1778681017.653533708] [pcd_subsriber_node]: Inference time: 15.221s
[INFO] [1778681017.659045323] [pcd_subsriber_node]: Predicted translation: [-0.66154826 -0.17266974  0.14786808], quaternion: (-0.9793963164618764, 0.008785210428868484, 0.2017483591233684, 0.0018096877237298594)
[INFO] [1778681017.660737212] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-2.87067282e-02  9.99511668e-01  1.23508276e-02 -1.99372229e-02]
 [ 9.18140023e-01  3.12509087e-02 -3.95021996e-01  4.33337079e-02]
 [-3.95215005e-01  3.19952842e-10 -9.18588698e-01 -1.28205690e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778681017.665722348] [pcd_subsriber_node]: Placement pose translation: [-0.01993722  0.04333371 -0.01282057], quaternion: (0.6816580923955353, 0.7033040389911775, -0.140416495764379, 0.14487540000851926)
[INFO] [1778681017.668545085] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/8-4_grasp.ply
[INFO] [1778681021.054257038] [pcd_subsriber_node]: Inference complete. Shutting down.
```