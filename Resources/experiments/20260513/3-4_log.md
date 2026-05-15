## grasp pose
```
header:
  stamp:
    sec: 1778678670
    nanosec: 413676198
  frame_id: base_link
pose:
  position:
    x: -0.8067144751548767
    y: 0.17186835408210754
    z: 0.15956364572048187
  orientation:
    x: 0.9930198199616932
    y: 0.06468220785577607
    z: -0.0984212887552304
    w: 0.006410855629976919
```
## placement pose
```
header:
  stamp:
    sec: 1778678670
    nanosec: 429666769
  frame_id: placement_link
pose:
  position:
    x: -0.04389239175921661
    y: 0.013898075912259222
    z: -0.008279595502198145
  orientation:
    x: 0.6557981809664367
    y: 0.7484656951286583
    z: -0.06499820049127794
    w: 0.07418276747049889
```
## vmf python log
```
[INFO] [1778678661.036163477] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778678661.138447257] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778678661.247033753] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778678661.248466333] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778678661, nanosec=150781790), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923690691185619, y=0.1334462630238533, z=0.6858221877773213), rotation=geometry_msgs.msg.Quaternion(x=0.6866697347297395, y=0.7248118911495134, z=-0.055945914530288646, w=-0.0015660465590053596)))
[INFO] [1778678661.253080175] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 3-4
Bounding Box center: [-0.12510735  0.12829168  0.05699786]
Point-cloud center of gravity: [-0.11626688  0.1277076   0.07668862]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/3-4_poses.ply
[INFO] [1778678670.407497805] [pcd_subsriber_node]: Inference time: 5.046s
[INFO] [1778678670.413268251] [pcd_subsriber_node]: Predicted translation: [-0.8067145   0.17186835  0.15956365], quaternion: (0.9930198199616932, 0.06468220785577607, -0.0984212887552304, 0.006410855629976919)
[INFO] [1778678670.415691095] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-1.28851334e-01  9.91328356e-01  2.57951118e-02 -4.38923918e-02]
 [ 9.72041450e-01  1.31407948e-01 -1.94595702e-01  1.38980759e-02]
 [-1.96297929e-01  3.03641823e-09 -9.80544329e-01 -8.27959550e-03]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778678670.429300104] [pcd_subsriber_node]: Placement pose translation: [-0.04389239  0.01389808 -0.0082796 ], quaternion: (0.6557981809664367, 0.7484656951286583, -0.06499820049127794, 0.07418276747049889)
[INFO] [1778678670.433080517] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/3-4_grasp.ply
[INFO] [1778678672.516551388] [pcd_subsriber_node]: Inference complete. Shutting down.
```