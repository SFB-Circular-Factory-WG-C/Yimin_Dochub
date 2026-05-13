## grasp pose
```
header:
  stamp:
    sec: 1778601776
    nanosec: 790462497
  frame_id: base_link
pose:
  position:
    x: -0.6706932187080383
    y: 0.1528506875038147
    z: 0.16300545632839203
  orientation:
    x: 0.9863858445622162
    y: 0.04240842839677794
    z: -0.1587385064335756
    w: 0.006824765449175726
```
## placement pose
```
header:
  stamp:
    sec: 1778601776
    nanosec: 801425664
  frame_id: placement_link
pose:
  position:
    x: -0.015269079618686072
    y: 0.0003337159737747397
    z: 0.0028926247407403127
  orientation:
    x: 0.6075697014134308
    y: 0.7782124176316023
    z: -0.09777584205233454
    w: 0.12523727751483635
```
## vmf python log
```
/usr/lib/python3/dist-packages/scipy/__init__.py:146: UserWarning: A NumPy version >=1.17.3 and <1.25.0 is required for this version of SciPy (detected version 1.26.4
  warnings.warn(f"A NumPy version >={np_minversion} and <{np_maxversion}"
2.7.0
Cuda available:  True
Cuda device number:  1
Current data path: ..
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/backbone/pointvit_inv.py:25: FutureWarning: `torch.cuda.amp.custom_fwd(args...)` is deprecated. Please use `torch.amp.custom_fwd(args..., device_type='cuda')` instead.
  def forward(ctx, x, blocks, pos_embed=None, alpha=0., lambd=1., num_cached = 0):
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/backbone/pointvit_inv.py:60: FutureWarning: `torch.cuda.amp.custom_bwd(args...)` is deprecated. Please use `torch.amp.custom_bwd(args..., device_type='cuda')` instead.
  def backward(ctx, dy):  # pragma: no cover
../dataset/vmf_data/data*
Namespace(devices=1, debug=False, learning_rate=0.0001, learning_rate_score=0.0003, learning_rate_decay=0.0005, run_finetuning=False, max_epochs=20000, flow_finetune=0, batch_size=2, epoch_length=1250, learning_rates=[1e-05, 2e-05, 5e-05, 0.0001, 0.0002, 0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1], camera_num=2, gradient_accumulation_steps=1, eval=True, ckpt='epoch=112-step=86558', data_root_dir=['../vmf_data'], num_workers=1, image_size=[480, 640], pcd_with_rgb=False, scale=0.875, seed=42, experiment=None, dataset='mgn', prob_baseline='lh', certainty_budget='constant', entropy_weight=1e-06, flow_layers=8, point_backbone='pointnext-b', hidden_feat_flow=512, embedding_dim=240, backbone='clip', flow_type='resflow', epochs=10, learning_rate_flow='3e-4', resume=True, use_best_model=False)
Seed set to 42
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/../vmf_contact_main/cfgs/vmfcontact/*.yaml
Loading /home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/../vmf_contact_main/cfgs/vmfcontact/pointnext-b.yaml
/usr/local/lib/python3.10/dist-packages/pytorch_lightning/core/saving.py:197: Found keys that are not in the model state dict but in the checkpoint: ['uncertainty_estimator.vmfoutput.prior.sufficient_statistics', 'uncertainty_estimator.vmfoutput.prior.evidence']
[INFO] [1778601736.234832083] [pcd_subsriber_node]: Waiting for depth + camera info...
[INFO] [1778601736.337369803] [pcd_subsriber_node]: Shape of the depth image: (720, 1280)
[INFO] [1778601736.439134025] [pcd_subsriber_node]: Shape of pcd: (100000, 3)
[INFO] [1778601736.440624153] [pcd_subsriber_node]: Transform:
geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1778601736, nanosec=296443344), frame_id='base_link'), child_frame_id='orbbec_femto_mega_link', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-0.6923540942070415, y=0.13345014823403928, z=0.6858063246276168), rotation=geometry_msgs.msg.Quaternion(x=0.6866714054083523, y=0.7248110347969371, z=-0.05593737812247543, w=-0.0015344575246276824)))
[INFO] [1778601736.445218348] [pcd_subsriber_node]: Shape of pcd_base: (100000, 3)
Enter a name for the visualization: Name convention is "<input>_grasp.ply" and "<input>_poses.ply" 
 2-3
Bounding Box center: [0.02486044 0.12246411 0.05583204]
Point-cloud center of gravity: [0.02825572 0.15319058 0.06239363]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/2-3_poses.ply
[INFO] [1778601776.783493954] [pcd_subsriber_node]: Inference time: 4.488s
[INFO] [1778601776.789764747] [pcd_subsriber_node]: Predicted translation: [-0.6706932   0.15285069  0.16300546], quaternion: (0.9863858445622162, 0.04240842839677794, -0.1587385064335756, 0.006824765449175726)
[INFO] [1778601776.796857550] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-2.30349372e-01  9.70126973e-01  7.61111271e-02 -1.52690796e-02]
 [ 9.21146240e-01  2.42597896e-01 -3.04361489e-01  3.33715974e-04]
 [-3.13733697e-01  1.47076351e-09 -9.49511051e-01  2.89262474e-03]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778601776.801183384] [pcd_subsriber_node]: Placement pose translation: [-0.01526908  0.00033372  0.00289262], quaternion: (0.6075697014134308, 0.7782124176316023, -0.09777584205233454, 0.12523727751483635)
[INFO] [1778601776.807692265] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/2-3_grasp.ply
[INFO] [1778601792.057631381] [pcd_subsriber_node]: Inference complete. Shutting down.
```