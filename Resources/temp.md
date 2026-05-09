- b. Resolution of the point cloud not high enough?
- e.g. Here snap a new pcd for front high th = 0.8 w/o bounds with 2400000 points and compare with the previous pcd with 40000 points.
    - Example 1
    - pcd (240000,3)
    - process time ~370s
![img_2-9](issues_260311/img_2-9.png)
![img_2-10](issues_260311/img_2-10.png)
    - Example 2
    - pcd (40000, 3)
    - process time ~335s
![img_2-11](issues_260311/img_2-11.png)
![img_2-12](issues_260311/img_2-12.png)
    - As can be seen from the screenshots, there were no major difference of the estimated pose positions. The chosen poses were slightly different.