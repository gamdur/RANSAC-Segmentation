# RANSAC-Segmentation

This is an iterative program that parses pcd point clouds and segments them into cylinders and planes.
To run the program go to cylinderSegmentation/build and run ./cylinder_segmentation <your_point_cloud.pcd>.

Here are a few parameters you can modify in cylinderSegmentation/cylinder_segmentation:
1) Parameters for RANSAC model of a plane:
    Line 326 - seg.setNormalDistanceWeight (0.1);
    Line 328 - seg.setMaxIterations (100);
    Line 329 - seg.setDistanceThreshold (0.02);
2) Parameters for RANSAC model of a Cylinder:    
    Line 365 - seg.setNormalDistanceWeight (0.1);
    Line 366 - seg.setMaxIterations (10000);
    Line 367 - seg.setDistanceThreshold (0.05);
    Line 370 - seg.setRadiusLimits (0, 10.0);
3) Threshold for a cylinder to be a plane. This is determined by fitting both a cylinder and a plane to a cloud. 
   The threshold is the ration between the cylinder cloud size and the plane cloud size.
    Line 396 - threshold = 0.2;
4) Threshold for to clouds to be merged. This is determined the same shape to both clouds. 
   The threshold is the ration between these two clouds sizes.
    Line 462 - threshold = 0.79;
5) Threshold for stop iterating. This is determined by the size of the clouds found by RANSAC in the current iteration.
    Line 564 - THRESHOLD = 500;
