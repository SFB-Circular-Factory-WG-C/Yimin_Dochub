# 3-1 log
## grasp pose
```
header:
  stamp:
    sec: 1778250358
    nanosec: 203713999
  frame_id: base_link
pose:
  position:
    x: -0.8078407645225525
    y: 0.14254513382911682
    z: 0.1422252506017685
  orientation:
    x: 0.9730110826017015
    y: 0.12000979492932667
    z: -0.12663537434949582
    w: 0.15103166627780054
```
## placement pose
```
header:
  stamp:
    sec: 1778250358
    nanosec: 208933551
  frame_id: placement_link
pose:
  position:
    x: -0.032366069918883134
    y: 0.013683886620832642
    z: -0.016802282331996404
  orientation:
    x: 0.6251464132437393
    y: 0.7552118124795092
    z: 0.011595644226532326
    w: 0.19675523203939008
```
## vmf python log
```
/usr/lib/python3/dist-packages/scipy/__init__.py:146: UserWarning: A NumPy version >=1.17.3 and <1.25.0 is required for this version of SciPy (detected version 1.26.4
  warnings.warn(f"A NumPy version >={np_minversion} and <{np_maxversion}"
2.7.0
Cuda available:  True
Cuda device number:  1
Current data path: ..
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/backbone/pointvit_inv.py:25: FutureWarning: `torch.cuda.amp.custom_fwd(args...)` is deprecated. Please use `torch.amp.custom_fwd(args..., device_type='cuda')` instead.
  def forward(ctx, x, blocks, pos_embed=None, alpha=0., lambd=1., num_cached = 0):
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/backbone/pointvit_inv.py:60: FutureWarning: `torch.cuda.amp.custom_bwd(args...)` is deprecated. Please use `torch.amp.custom_bwd(args..., device_type='cuda')` instead.
  def backward(ctx, dy):  # pragma: no cover
../dataset/vmf_data/data*
Namespace(devices=1, debug=False, learning_rate=0.0001, learning_rate_score=0.0003, learning_rate_decay=0.0005, run_finetuning=False, max_epochs=20000, flow_finetune=0, batch_size=2, epoch_length=1250, learning_rates=[1e-05, 2e-05, 5e-05, 0.0001, 0.0002, 0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1], camera_num=2, gradient_accumulation_steps=1, eval=True, ckpt='epoch=112-step=86558', data_root_dir=['../vmf_data'], num_workers=1, image_size=[480, 640], pcd_with_rgb=False, scale=0.875, seed=42, experiment=None, dataset='mgn', prob_baseline='lh', certainty_budget='constant', entropy_weight=1e-06, flow_layers=8, point_backbone='pointnext-b', hidden_feat_flow=512, embedding_dim=240, backbone='clip', flow_type='resflow', epochs=10, learning_rate_flow='3e-4', resume=True, use_best_model=False)
Seed set to 42
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/../vmf_contact_main/cfgs/vmfcontact/*.yaml
Loading /home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/../vmf_contact_main/cfgs/vmfcontact/pointnext-b.yaml
/usr/local/lib/python3.10/dist-packages/pytorch_lightning/core/saving.py:197: Found keys that are not in the model state dict but in the checkpoint: ['uncertainty_estimator.vmfoutput.prior.sufficient_statistics', 'uncertainty_estimator.vmfoutput.prior.evidence']
[INFO] [1778250328.839096104] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778250328.941646598] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778250329.040067550] [pcd_subsriber_node]: Shape of pcd: (40000, 3)
[INFO] [1778250329.041531907] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778250328, nanosec=966274930), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.692423057536672, y=0.1334401381725037, z=0.6857612393219031), rotation=geometry_msgs.msg.Quaternion(x=0.6866625512202491, y=0.7248195941637671, z=-0.05593503273603328, w=-0.0015390843997986987)))
[INFO] [1778250329.044147267] [pcd_subsriber_node]: Shape of pcd_base: (40000, 3)
Bounding Box center: [-0.1245267   0.11173082  0.05128616]
Point-cloud center of gravity: [-0.11475463  0.10716055  0.0647689 ]
/home/USERNAME/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
[INFO] [1778250358.193923006] [pcd_subsriber_node]: Inference time: 29.148s
[INFO] [1778250358.203382086] [pcd_subsriber_node]: Predicted translation: [-0.80784076  0.14254513  0.14222525], quaternion: (0.9730110826017015, 0.12000979492932667, -0.12663537434949582, 0.15103166627780054)
[INFO] [1778250358.204723367] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-0.14095869  0.93967297  0.3116817  -0.03236607]
 [ 0.94879897  0.21811502 -0.22848731  0.01368389]
 [-0.28268582  0.26351601 -0.92230588 -0.01680228]
 [ 0.          0.          0.          1.        ]]
[INFO] [1778250358.208721770] [pcd_subsriber_node]: Placement pose translation: [-0.03236607  0.01368389 -0.01680228], quaternion: (0.6251464132437393, 0.7552118124795092, 0.011595644226532326, 0.19675523203939008)
[INFO] [1778250358.210007017] [pcd_subsriber_node]: Place pose published.
[INFO] [1778250739.637949614] [pcd_subsriber_node]: Inference complete. Shutting down.
```
## moveit
```
[moveit2_iface-3] [ERROR] [1778250506.321224169] [moveit2_iface]: IK solution not found for target pose.
[moveit2_iface-3] [INFO] [1778250506.423615771] [moveit2_iface]: Received new target pose for servo control.
[moveit2_iface-3] [WARN] [1778250506.424110058] [moveit2_iface]: IK solution found but is in collision or invalid.
[moveit2_iface-3] [WARN] [1778250506.424256904] [moveit2_iface]: Colliding links: forearm_link, orbbec_femto_mega_link
```