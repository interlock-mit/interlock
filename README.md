# interlock

This is an experimental implementation of a runtime monitor for self-driving cars that verifies aspects of perception in both LiDAR and vision. In each example, a controller passes a certificate to the monitor, which uses the certificate to either confirm or deny the safety of the situation. 

### LiDAR
The controller filters snow particles from the LiDAR point cloud and generates a certificate containing points that are far enough from the source to ensure no collisions will happen in the near future. 

The monitor checks that the LiDAR points in the certificate satisfy two conditions: sufficient spread and sufficient density. To achieve sufficient spread, certificate must span the size of one lane in front of the car, both vertically and horizontally. 

### Vision + LiDAR
The controller takes points from the camera image's lane lines and computes the LiDAR points that correspond to those pixels. It generates a certificate containing those points and the ground plane (computed from the LiDAR point cloud).

The monitor checks that the LiDAR points corresponding to lane line points are on the ground plane. 

### Vision
The controller generates a certificate containing a proposed pair of lane lines (given as polynomials in a top-down view), a transformation matrix for converting from top-down to camera view, an image of the road, and a set of color filtering thresholds appropriate for the current lighting conditions

The monitor checks that:
1. The lane lines from the polynomials are adequately spaced apart and parallel
2. The polynomials match closely with the image (using the transformation matrix to get the corresponding points in the camera view and the color filtering thresholds for processing the image)