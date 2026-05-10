## grasp pose
```
header:
  stamp:
    sec: 1778253790
    nanosec: 142485200
  frame_id: placement_link
pose:
  position:
    x: 0.0023875502799537117
    y: 0.005579131335780696
    z: 0.0032892711702221245
  orientation:
    x: 0.6892510351999955
    y: 0.7109178044248436
    z: -0.09727444377791623
    w: 0.1003322899927609
```
## placement pose
```
header:
  stamp:
    sec: 1778253790
    nanosec: 142485200
  frame_id: placement_link
pose:
  position:
    x: 0.0023875502799537117
    y: 0.005579131335780696
    z: 0.0032892711702221245
  orientation:
    x: 0.6892510351999955
    y: 0.7109178044248436
    z: -0.09727444377791623
    w: 0.1003322899927609
```
## vmf python log
```
[INFO] [1778253771.131856492] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778253771.234432018] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778253771.331004598] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778253771.332524295] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778253771, nanosec=204307829), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923710638900152, y=0.1334478147257255, z=0.6857597712033648), rotation=geometry_msgs.msg.Quaternion(x=0.6866595200174109, y=0.7248249690422002, z=-0.05590284448010538, w=-0.0015296409597028394)))
[INFO] [1778253771.335067544] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [ 0.03281326 -0.06078967  0.05233116]
Point-cloud center of gravity: [ 0.03271621 -0.03189675  0.05921663]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778253790.115994397] [pcd_subsriber_node]: Inference time: 18.778s
[INFO] [1778253790.125800120] [pcd_subsriber_node]: Predicted translation: [-0.66158843 -0.04856591  0.16006316], quaternion: (0.9897371995983639, 0.029856145763149322, -0.13968225268212894, 0.004213618041958885)
[INFO] [1778253790.131295337] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-2.97328841e-02  9.99521249e-01  8.56300054e-03  2.38755028e-03]
 [ 9.60482129e-01  3.09413859e-02 -2.76616535e-01  5.57913134e-03]
 [-2.76749045e-01  1.99817607e-09 -9.60942209e-01  3.28927117e-03]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778253790.142184391] [pcd_subsriber_node]: Placement pose translation: [0.00238755 0.00557913 0.00328927], quaternion: (0.6892510351999955, 0.7109178044248436, -0.09727444377791623, 0.1003322899927609)
[INFO] [1778253790.147952992] [pcd_subsriber_node]: Place pose published.
[INFO] [1778253806.252855296] [pcd_subsriber_node]: Inference complete. Shutting down.
```