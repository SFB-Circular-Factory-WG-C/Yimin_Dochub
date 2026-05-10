## grasp pose
```
header:
  stamp:
    sec: 1778254547
    nanosec: 282505541
  frame_id: base_link
pose:
  position:
    x: -0.7952154278755188
    y: 0.019961630925536156
    z: 0.14151716232299805
  orientation:
    x: 0.9746086455610096
    y: 0.0495307510436937
    z: -0.21808679559568617
    w: 0.011083423910847545
```
## placement pose
```
header:
  stamp:
    sec: 1778254547
    nanosec: 298189130
  frame_id: placement_link
pose:
  position:
    x: -0.040280706817847865
    y: 0.014923709225417858
    z: -0.015189443570290118
  orientation:
    x: 0.6691454834040709
    y: 0.7103236089657878
    z: -0.14973373766235573
    w: 0.15894810596987805
```
## vmf python log
```
[INFO] [1778254527.781517652] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778254527.883583117] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778254527.977703728] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778254527.979183678] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778254527, nanosec=885139962), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923877742534684, y=0.1334364502043889, z=0.6857551674095301), rotation=geometry_msgs.msg.Quaternion(x=0.686661439633211, y=0.7248220257834178, z=-0.05591796104237759, w=-0.0015099314327691937)))
[INFO] [1778254527.981767118] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [-0.11480772 -0.02107284  0.04661067]
Point-cloud center of gravity: [-0.10207217 -0.02074409  0.06480254]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778254547.276906616] [pcd_subsriber_node]: Inference time: 19.292s
[INFO] [1778254547.282122873] [pcd_subsriber_node]: Predicted translation: [-0.7952154   0.01996163  0.14151716], quaternion: (0.9746086455610096, 0.0495307510436937, -0.21808679559568617, 0.011083423910847545)
[INFO] [1778254547.283630984] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-5.39596461e-02  9.98219447e-01  2.54218800e-02 -4.02807068e-02]
 [ 9.03019929e-01  5.96482526e-02 -4.25437641e-01  1.49237092e-02]
 [-4.26196516e-01 -4.03592659e-10 -9.04630661e-01 -1.51894436e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778254547.297892961] [pcd_subsriber_node]: Placement pose translation: [-0.04028071  0.01492371 -0.01518944], quaternion: (0.6691454834040709, 0.7103236089657878, -0.14973373766235573, 0.15894810596987805)
[INFO] [1778254547.300348493] [pcd_subsriber_node]: Place pose published.
[INFO] [1778254610.909153248] [pcd_subsriber_node]: Inference complete. Shutting down.
```