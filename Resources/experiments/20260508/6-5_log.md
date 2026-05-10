## grasp pose
```
header:
  stamp:
    sec: 1778255618
    nanosec: 574604576
  frame_id: base_link
pose:
  position:
    x: -0.8070954084396362
    y: -0.04910486936569214
    z: 0.15485557913780212
  orientation:
    x: 0.9861674495678252
    y: 0.038757080035128455
    z: -0.16103290842163018
    w: 0.0063287096088122516
```
## placement pose
```
header:
  stamp:
    sec: 1778255618
    nanosec: 585523322
  frame_id: placement_link
pose:
  position:
    x: -0.01409457055412476
    y: -0.01797355579373694
    z: 0.001775000352363193
  orientation:
    x: 0.6584705127710304
    y: 0.735149598912864
    z: -0.10752273637007988
    w: 0.12004379281246279
```
## vmf python log
```
[INFO] [1778255590.379732691] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778255590.482270666] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778255590.578182096] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778255590.579685312] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778255590, nanosec=468393852), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.692376357908506, y=0.13344155096878788, z=0.68579311818818), rotation=geometry_msgs.msg.Quaternion(x=0.6866719985762518, y=0.7248111594966967, z=-0.05592797953624599, w=-0.0015525931029923303)))
[INFO] [1778255590.582197968] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [-0.07957027 -0.08216591  0.04631138]
Point-cloud center of gravity: [-0.0995753  -0.04534662  0.05784978]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778255618.569176500] [pcd_subsriber_node]: Inference time: 27.984s
[INFO] [1778255618.574220756] [pcd_subsriber_node]: Predicted translation: [-0.8070954  -0.04910487  0.15485558], quaternion: (0.9861674495678252, 0.038757080035128455, -0.16103290842163018, 0.0063287096088122516)
[INFO] [1778255618.580734626] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-1.04012147e-01  9.93963567e-01  3.48991874e-02 -1.40945706e-02]
 [ 9.42333825e-01  1.09710897e-01 -3.16181177e-01 -1.79735558e-02]
 [-3.18101406e-01  1.19063426e-09 -9.48056698e-01  1.77500035e-03]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778255618.585185967] [pcd_subsriber_node]: Placement pose translation: [-0.01409457 -0.01797356  0.001775  ], quaternion: (0.6584705127710304, 0.735149598912864, -0.10752273637007988, 0.12004379281246279)
[INFO] [1778255618.592680637] [pcd_subsriber_node]: Place pose published.
[INFO] [1778255691.833177353] [pcd_subsriber_node]: Inference complete. Shutting down.
```