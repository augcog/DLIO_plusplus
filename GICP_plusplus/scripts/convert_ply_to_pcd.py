#!/usr/bin/env python3
"""
Convert PLY format point cloud to PCD format for DLIO localization.
"""
import sys

try:
    import open3d as o3d
except ImportError:
    print("Error: open3d is required. Install with: pip install open3d")
    sys.exit(1)

def convert_ply_to_pcd(input_ply, output_pcd):
    """Convert PLY file to PCD format."""
    print(f"Loading PLY file: {input_ply}")
    pcd = o3d.io.read_point_cloud(input_ply)

    num_points = len(pcd.points)
    print(f"Loaded {num_points} points")

    if num_points == 0:
        print("Error: No points loaded from PLY file!")
        return False

    print(f"Saving PCD file: {output_pcd}")
    o3d.io.write_point_cloud(output_pcd, pcd, write_ascii=False)
    print(f"Successfully converted PLY to PCD!")
    return True

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 convert_ply_to_pcd.py <input.ply> <output.pcd>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    if convert_ply_to_pcd(input_file, output_file):
        sys.exit(0)
    else:
        sys.exit(1)
