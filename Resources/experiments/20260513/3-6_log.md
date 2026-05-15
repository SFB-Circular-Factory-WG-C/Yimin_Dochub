## grasp pose
```
header:
  stamp:
    sec: 1778679076
    nanosec: 288150531
  frame_id: base_link
pose:
  position:
    x: -0.8116928339004517
    y: 0.13997094333171844
    z: 0.1598818302154541
  orientation:
    x: 0.9814443399650536
    y: 0.18213980422885792
    z: -0.058927939787672065
    w: 0.010936049488450736
```
## placement pose
```
header:
  stamp:
    sec: 1778679076
    nanosec: 291982302
  frame_id: placement_link
pose:
  position:
    x: -0.01488096910315967
    y: 0.012601520103545316
    z: -0.010494822460334646
  orientation:
    x: 0.6389516841749899
    y: 0.7669084991924132
    z: -0.03836397366688454
    w: 0.04604676623685097
```
## vmf python log
```
[INFO] [1778679066.719021726] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778679066.821165156] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778679066.922333681] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778679066.923807157] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778679066, nanosec=816889814), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923636968771828, y=0.13346708051123338, z=0.68580599118435), rotation=geometry_msgs.msg.Quaternion(x=0.6866641696328893, y=0.7248178070645123, z=-0.05593840915532796, w=-0.001535931553604547)))
[INFO] [1778679066.928443075] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 3-6
Bounding Box center: [-0.12633386  0.12063999  0.06163482]
Point-cloud center of gravity: [-0.11633465  0.12540296  0.07711849]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/3-6_poses.ply
[INFO] [1778679076.283754714] [pcd_subsriber_node]: Inference time: 4.881s
[INFO] [1778679076.287815579] [pcd_subsriber_node]: Predicted translation: [-0.81169283  0.13997094  0.15988183], quaternion: (0.9814443399650536, 0.18213980422885792, -0.058927939787672065, 0.010936049488450736)
[INFO] [1778679076.289186908] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-1.79240891e-01  9.83568033e-01  2.16018605e-02 -1.48809691e-02]
 [ 9.76501934e-01  1.80537917e-01 -1.17686636e-01  1.26015201e-02]
 [-1.19652770e-01  1.02960518e-09 -9.92815852e-01 -1.04948225e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778679076.291832763] [pcd_subsriber_node]: Placement pose translation: [-0.01488097  0.01260152 -0.01049482], quaternion: (0.6389516841749899, 0.7669084991924132, -0.03836397366688454, 0.04604676623685097)
[INFO] [1778679076.292900596] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/3-6_grasp.ply
[INFO] [1778679079.339507866] [pcd_subsriber_node]: Inference complete. Shutting down.
```