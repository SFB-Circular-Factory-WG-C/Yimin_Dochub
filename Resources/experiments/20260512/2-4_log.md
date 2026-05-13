## grasp pose
```
header:
  stamp:
    sec: 1778602228
    nanosec: 609268429
  frame_id: base_link
pose:
  position:
    x: -0.6917933225631714
    y: 0.1324819028377533
    z: 0.15910474956035614
  orientation:
    x: -0.004385774925074531
    y: 0.997285935642052
    z: 0.0003232070147852328
    w: 0.073494374520019
```
## placement pose
```
header:
  stamp:
    sec: 1778602228
    nanosec: 623367951
  frame_id: placement_link
pose:
  position:
    x: 0.03710603118961309
    y: 0.024997481775887676
    z: -0.0017118282487992853
  orientation:
    x: 0.7883461206501032
    y: 0.610826380075131
    z: -0.058096682684702736
    w: 0.0450144745751595
```
## vmf python log
```
[INFO] [1778602219.167011682] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778602219.269262308] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778602219.377633994] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778602219.379497029] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778602219, nanosec=230472517), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923804983708485, y=0.13342203800894803, z=0.685808143317618), rotation=geometry_msgs.msg.Quaternion(x=0.6866484373215307, y=0.72483192253148, z=-0.055948452737522684, w=-0.0015421541738160386)))
[INFO] [1778602219.384102521] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 2-4
Bounding Box center: [0.06827164 0.0538547  0.05701355]
Point-cloud center of gravity: [0.01560521 0.15232472 0.0626196 ]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/2-4_poses.ply
[INFO] [1778602228.582350833] [pcd_subsriber_node]: Inference time: 5.148s
[INFO] [1778602228.608933922] [pcd_subsriber_node]: Predicted translation: [-0.6917933   0.1324819   0.15910475], quaternion: (-0.004385774925074531, 0.997285935642052, 0.0003232070147852328, 0.073494374520019)
[INFO] [1778602228.610328495] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[ 2.47031819e-01  9.68315593e-01 -3.66085301e-02  3.71060312e-02]
 [ 9.57854837e-01 -2.49729660e-01 -1.41947939e-01  2.49974818e-02]
 [-1.46592647e-01  6.73015671e-11 -9.89196956e-01 -1.71182825e-03]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778602228.622731227] [pcd_subsriber_node]: Placement pose translation: [ 0.03710603  0.02499748 -0.00171183], quaternion: (0.7883461206501032, 0.610826380075131, -0.058096682684702736, 0.0450144745751595)
[INFO] [1778602228.624456338] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/2-4_grasp.ply
[INFO] [1778602244.370815674] [pcd_subsriber_node]: Inference complete. Shutting down.
```