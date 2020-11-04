# from plyfile import PlyData, PlyElement
# with open('lidar002310.ply', 'rb') as f:
#     plydata = PlyData.read(f)
#     print(plydata.elements)

# import numpy as np
# import open3d
# def main():
#     cloud = open3d.read_point_cloud("lidar002310.ply") # Read the point cloud
#     print(np.asarray(cloud.points))
#     open3d.draw_geometries([cloud]) # Visualize the point cloud     

# if __name__ == "__main__":
#     main()

with open('lidar002310.ply', 'r') as f:
    while True:
        line = f.readline().split(' ')
        if not line:
            break
        if len(line) > 4 and line[4] != "0.0":
            print(line)
        # elif len(line) < 4:
        #     print(line)
        