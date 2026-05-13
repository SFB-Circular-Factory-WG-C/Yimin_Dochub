# Experiments 2026.05.12
## Test 2 variant of 2026.05.08
- Added a wedge under the angle grinder
![View2-z](#TODO.png)

|No.|Result|Description|Poses|Grasp|Log|Record|
|--|--|--|--|--|--|--|
|3|Failed|Item fell during movement|[2-3](2-3_poses.ply)|[2-3](2-3_grasp.ply)|[2-3](2-3_log.md)|[2-3](2-3.mp4)|
|4|Partially successful|Manual intervention, *placement upside down|[2-4](2-4_poses.ply)|[2-4](2-4_grasp.ply)|[2-4](2-4_log.md)|[2-4](2-4.mp4)|
|5|Partially successful|Manual intervention|[2-5](2-5_poses.ply)|[2-5](2-5_grasp.ply)|[2-5](2-5_log.md)|[2-5](2-5.mp4)|
|6|Partially successful|Manual intervention|[2-6](2-6_poses.ply)|[2-6](2-6_grasp.ply)|[2-6](2-6_log.md)|[2-6](2-6.mp4)|
|7|Partially successful|Manual intervention|[2-7](2-7_poses.ply)|[2-7](2-7_grasp.ply)|[2-7](2-7_log.md)|[2-7](2-7.mp4)|

- Conclusion: 
    - After adding the wedge, Test2 variant partially succeeded 4/5. 
    - For successful placement, manual intervention was necessary.
    - *The method proved to be applicable even when the angle grinder was grasped in the other direction
        - There is a filter in the algorithm which should filter out poses whose y-axis does not point to the higher end of the tool
        - However, in some cases it might be hard to tell which side is the higher end (due to the wedge)