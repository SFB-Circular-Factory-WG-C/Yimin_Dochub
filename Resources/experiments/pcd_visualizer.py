import argparse
import open3d as o3d

parser = argparse.ArgumentParser(
    description="Visualize a point cloud file"
)

parser.add_argument(
    "file",
    help="Path to point cloud file"
)

args = parser.parse_args()

pcd = o3d.io.read_point_cloud(args.file)

o3d.visualization.draw_geometries([pcd])

# # top-down view for all orientations in position D
# o3d.visualization.draw_geometries(
#     [pcd],
#     front = [ 0.24432865056963404, -0.063045399097209631, 0.96764083634553966 ],
#     lookat = [ 0.082154750954158903, -0.046233946246530523, 0.01714465772493402 ],
#     up = [ 0.96896772409211118, -0.022700537163399686, -0.24614271323816939 ],
#     zoom = 0.73999999999999999
#     )


# # top-down view for all positions in orientation 1
# o3d.visualization.draw_geometries(
#     [pcd],
#     front = [ 0.24432865056963404, -0.063045399097209631, 0.96764083634553966 ],
#     lookat = [ -0.024879478109996706, -0.019525280737142207, 0.045910896377619019 ],
#     up = [0.96896772409211118, -0.022700537163399686, -0.24614271323816939],
#     zoom = 0.73999999999999999
#     )
