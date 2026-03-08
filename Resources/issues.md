# Issues
## 11.03.2026 Biweekly Meetup
1. The gripper went to a deeper z-position, which then led to the crash with the desk and thus an emergency stop.
- The pose was given by the vMF-Algorithm.
- **TODO** example here pcd 9274 th=0.8 w/o bounds. **Photos from different perspectives or show this position in real**
- Due to the bigger gripper? Any solution to shift the gripper a bit in approach direction?

2. The chosen pose was not always ideal and usable, even if it was the same object in different position.
- **TODO** example here pcd 6177 th=0.9 without bounds
    - results focused on top of the angle grinder
- **TODO** example here pcd 6072 th=0.9 without bounds
    - Even if the threshold was set to 0.9, the result still fell outside of the item.
- Other results
    - a. Pointcloud without center shift delievered better results
    - The pose is based on the link "base_link", not an extra link e.g. "pcd_center" , because the shift of the coordination axis of the pointcloud will require extra effort in caliberation but at the same time the result would not be any better.
    - **TODO** example here pcd 9274 th = 0.8 w/o bounds vs th = 0.1 w/ bounds
    - pcd after crop + pose estimation
    - b. Resolution of the point cloud not high enough?
    - **TODO** example here snap a new pcd for front low+box th = 0.8 w/o bounds with 2400000 points and compare with the previous pcd with 40000 points(or even less)

3. The inference time varied between 80s to more than 200s. The system would not be usable with such a long processing time.


