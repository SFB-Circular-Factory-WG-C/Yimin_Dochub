## vmf python log
```
Bounding Box center: [-0.04703102 -0.19610496  0.0398386 ]
Point-cloud center of gravity: [ 0.02302153 -0.20326988  0.07353031]
/home/jetson/wbk_ur10_ws/src/vmf/vmf_contact_main/openpoints/models/layers/subsample.py:93: UserWarning: The torch.cuda.*DtypeTensor constructors are no longer recommended. It's best to use methods such as torch.tensor(data, dtype=*, device='cuda') to create tensors. (Triggered internally at /opt/pytorch/torch/csrc/tensor/python_tensor.cpp:78.)
  output = torch.cuda.IntTensor(B, npoint)
No OBB provided for visualization
Saved point cloud and poses to /home/jetson/wbk_ur10_ws/src/vmf/8-1_poses.ply
[INFO] [1778680553.656042250] [pcd_subsriber_node]: Inference time: 7.029s
[INFO] [1778680553.659625113] [pcd_subsriber_node]: Predicted translation: [-0.666612   -0.18409087  0.1436949 ], quaternion: (0.9797767112043109, 0.029883540427009664, -0.19775790493505294, 0.006031685412078895)
[INFO] [1778680553.661210314] [pcd_subsriber_node]: Grasp pose published.
cog_T_grasp:
[[-9.90042379e-02  9.94214472e-01  4.16633125e-02 -1.76959154e-02]
 [ 9.16378298e-01  1.07413547e-01 -3.85633564e-01  4.46162075e-02]
 [-3.87877703e-01 -2.23632335e-09 -9.21710849e-01 -1.39895475e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1778680553.678620805] [pcd_subsriber_node]: Placement pose translation: [-0.01769592  0.04461621 -0.01398955], quaternion: (0.6548459903173137, 0.7294053459035683, -0.13217396462226805, 0.1472230057044887)
[INFO] [1778680553.679969615] [pcd_subsriber_node]: Place pose published.
Saved point cloud to /home/jetson/wbk_ur10_ws/src/vmf/8-1_grasp.ply
[INFO] [1778680556.739213209] [pcd_subsriber_node]: Inference complete. Shutting down.
```