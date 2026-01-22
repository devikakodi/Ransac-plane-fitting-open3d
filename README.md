# Ransac-plane-fitting-open3d

## **Overview**

This project implements a **custom RANSAC-based plane fitting algorithm** for 3D point clouds using Open3D. The goal is to robustly identify dominant planar surfaces in noisy point cloud data by randomly sampling point triplets, estimating plane models, and selecting the plane with the maximum number of inliers.  
To improve plane coverage, the detected inliers are further expanded using a spatial neighborhood search, resulting in a more complete planar segmentation. The final output visualizes the detected plane in red against the remaining point cloud.

<img width="456" height="409" alt="Screenshot 2026-01-22 at 11 52 41 AM" src="https://github.com/user-attachments/assets/a59beb26-16b9-473a-81cd-19e574a98608" />


## **Key Features**

- Custom implementation of the **RANSAC algorithm** (no built-in RANSAC APIs used)
- Robust plane detection in noisy 3D point clouds
- KD-tree–based inlier expansion for improved plane coverage
- 3D visualization using **Open3D**
- Uses Open3D’s built-in demo point cloud (no external datasets required)

## Algorithm Overview

1. Load a 3D point cloud.
2. Randomly sample three points to estimate a candidate plane.
3. Compute the plane equation using the cross product.
4. Measure distances of all points to the plane.
5. Identify inliers within a distance threshold.
6. Repeat for multiple iterations and select the plane with the most inliers.
7. Expand the inlier set using a KD-tree neighborhood search.
8. Visualize the detected plane and remaining points.

## Results

- **Red points** represent the detected planar surface.
- Remaining points represent non-planar regions and outliers.
- The expanded inlier step improves plane continuity and robustness.
<img width="405" height="402" alt="Screenshot 2026-01-22 at 11 52 54 AM" src="https://github.com/user-attachments/assets/3d0d53a8-6b46-4e82-8169-e6b7f5e0485b" />
<img width="395" height="398" alt="Screenshot 2026-01-22 at 11 55 38 AM" src="https://github.com/user-attachments/assets/d1aade65-0951-4c61-bd68-863c2b1931b1" />

## Running the Code

1. Open `Main-code.ipynb` in **Jupyter Notebook** or **JupyterLab**.
2. Run all cells sequentially.
3. The notebook will:
   - Load a demo point cloud provided by Open3D
   - Perform RANSAC plane fitting
   - Expand inliers using a KD-tree
   - Visualize the detected plane in red

No additional datasets or configuration files are required.

## Notes

- The RANSAC logic is implemented **from scratch** to emphasize algorithmic understanding.
- Open3D is used only for point cloud loading and visualization.
- The approach is suitable for applications in **robot perception**, **scene understanding**, and **3D environment modeling**.

