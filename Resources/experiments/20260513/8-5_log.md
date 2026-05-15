## grasp pose
```
header:
  stamp:
    sec: 1778681283
    nanosec: 750998617
  frame_id: base_link
pose:
  position:
    x: -0.6529040336608887
    y: -0.2061682641506195
    z: 0.14774836599826813
  orientation:
    x: 0.9757194875624352
    y: 0.03602692032274449
    z: -0.21589348603358552
    w: 0.007971530042821817
```
## placement pose
```
header:
  stamp:
    sec: 1778681283
    nanosec: 755253009
  frame_id: placement_link
pose:
  position:
    x: -0.001633492042021123
    y: 0.04770210371791539
    z: -0.01314125851910708
  orientation:
    x: 0.6552474190433232
    y: 0.7238627520503318
    z: -0.14498392345035016
    w: 0.1601661573870294
```
## vmf python log
```
[INFO] [1778681272.433037194] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778681272.535269496] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778681272.639194743] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778681272.640644896] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778681272, nanosec=528049574), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923558030104082, y=0.1334515933317503, z=0.6858091513296607), rotation=geometry_msgs.msg.Quaternion(x=0.6866638121201822, y=0.7248177832542164, z=-0.05594302407033619, w=-0.0015389154911211545)))
[INFO] [1778681272.645188512] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 8-5
Bounding Box center: [-0.03744746 -0.20684853  0.04866697]
Point-cloud center of gravity: [ 0.03618281 -0.20631075  0.07111228]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/8-5_poses.ply
[INFO] [1778681283.746721088] [pcd_subsriber_node]: Inference time: 8.515s
[INFO] [1778681283.750650255] [pcd_subsriber_node]: Predicted translation: [-0.65290403 -0.20616826  0.14774837], quaternion: (0.9757194875624352, 0.03602692032274449, -0.21589348603358552, 0.007971530042821817)
[INFO] [1778681283.752075735] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-8.99952459e-02  9.95061426e-01  4.18759528e-02 -1.63349204e-03]
 [ 9.02175386e-01  9.92609617e-02 -4.19793884e-01  4.77021037e-02]
 [-4.21877325e-01 -7.74939779e-10 -9.06652987e-01 -1.31412585e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778681283.755050571] [pcd_subsriber_node]: Placement pose translation: [-0.00163349  0.0477021  -0.01314126], quaternion: (0.6552474190433232, 0.7238627520503318, -0.14498392345035016, 0.1601661573870294)
[INFO] [1778681283.756320655] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/8-5_grasp.ply
[INFO] [1778681286.219918611] [pcd_subsriber_node]: Inference complete. Shutting down.
```