## grasp pose
```
header:
  stamp:
    sec: 1778252951
    nanosec: 333763707
  frame_id: base_link
pose:
  position:
    x: -0.6864555478096008
    y: -0.01976795680820942
    z: 0.16058246791362762
  orientation:
    x: -0.9914426858076464
    y: -0.09538353833920235
    z: 0.06988321024947575
    w: 0.05531472044259282
```
## placement pose
```
header:
  stamp:
    sec: 1778252951
    nanosec: 356045491
  frame_id: placement_link
pose:
  position:
    x: -0.031709756135304926
    y: 0.01239945218958105
    z: 0.0039497371964918315
  orientation:
    x: 0.7026641265298914
    y: 0.7059176618455851
    z: -0.08910359928671593
    w: 0.00198206372223439
```
## vmf python log
```
[INFO] [1778252932.377904628] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778252932.480029446] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778252932.574366507] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778252932.575852709] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778252932, nanosec=476694573), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923578887345853, y=0.13341956111993275, z=0.6858564429316582), rotation=geometry_msgs.msg.Quaternion(x=0.6866424944632779, y=0.7248363220866274, z=-0.05596370911273204, w=-0.0015666025418136717)))
[INFO] [1778252932.578306084] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [ 0.00555247 -0.06914546  0.05258757]
Point-cloud center of gravity: [ 0.00897394 -0.03731731  0.05867789]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778252951.328376204] [pcd_subsriber_node]: Inference time: 18.747s
[INFO] [1778252951.333356395] [pcd_subsriber_node]: Predicted translation: [-0.68645555 -0.01976796  0.16058247], quaternion: (-0.9914426858076464, -0.09538353833920235, 0.06988321024947575, 0.05531472044259282)
[INFO] [1778252951.335515422] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-0.01251839  0.99239931 -0.12242146 -0.03170976]
 [ 0.99169284 -0.00335264 -0.12858505  0.01239945]
 [-0.12801816 -0.12301417 -0.98411328  0.00394974]
 [ 0.          0.          0.          1.        ]]
[INFO] [1778252951.355775209] [pcd_subsriber_node]: Placement pose translation: [-0.03170976  0.01239945  0.00394974], quaternion: (0.7026641265298914, 0.7059176618455851, -0.08910359928671593, 0.00198206372223439)
[INFO] [1778252951.357150494] [pcd_subsriber_node]: Place pose published.
[INFO] [1778252966.681907101] [pcd_subsriber_node]: Inference complete. Shutting down.
```