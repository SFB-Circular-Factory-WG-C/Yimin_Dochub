## grasp pose
```
header:
  stamp:
    sec: 1778254245
    nanosec: 146056824
  frame_id: base_link
pose:
  position:
    x: -0.799821138381958
    y: -0.01104678399860859
    z: 0.14148063957691193
  orientation:
    x: 0.9840977753719465
    y: 0.014423856050478662
    z: -0.17702200127619167
    w: 0.0025945999292864303
```
## placement pose
```
header:
  stamp:
    sec: 1778254245
    nanosec: 150372607
  frame_id: placement_link
pose:
  position:
    x: -0.03954848926740953
    y: 0.008191200140897359
    z: -0.012027457122392948
  orientation:
    x: 0.7071282479343651
    y: 0.6845627229395604
    z: -0.1272000175494977
    w: 0.1231408740843478
```
## vmf python log
```
[INFO] [1778254219.065543738] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778254219.167865977] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778254219.265346209] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778254219.266828911] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778254219, nanosec=140325293), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923747415617061, y=0.1334204665334176, z=0.6857784023965235), rotation=geometry_msgs.msg.Quaternion(x=0.6866616840399161, y=0.7248197185872682, z=-0.05594473986006113, w=-0.0015143647280954034)))
[INFO] [1778254219.269365023] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [-0.10507216 -0.05391288  0.04194274]
Point-cloud center of gravity: [-0.10604113 -0.04813774  0.06307346]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778254245.133246600] [pcd_subsriber_node]: Inference time: 25.861s
[INFO] [1778254245.145742319] [pcd_subsriber_node]: Predicted translation: [-0.79982114 -0.01104678  0.14148064], quaternion: (0.9840977753719465, 0.014423856050478662, -0.17702200127619167, 0.0025945999292864303)
[INFO] [1778254245.147256734] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[ 3.03880698e-02  9.99474309e-01 -1.12981478e-02 -3.95484893e-02]
 [ 9.36820296e-01 -3.24204062e-02 -3.48305584e-01  8.19120014e-03]
 [-3.48488778e-01  1.97374533e-10 -9.37313020e-01 -1.20274571e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778254245.150198458] [pcd_subsriber_node]: Placement pose translation: [-0.03954849  0.0081912  -0.01202746], quaternion: (0.7071282479343651, 0.6845627229395604, -0.1272000175494977, 0.1231408740843478)
[INFO] [1778254245.151518499] [pcd_subsriber_node]: Place pose published.
[INFO] [1778254263.317386100] [pcd_subsriber_node]: Inference complete. Shutting down.
```