# Bounding Box Disparity: 3D Metrics for Object Detection With Full Degree of Freedom
### Following metrics are part of the implementation:
- IoU_v: volumetric intersection over union
- v2v: volume-to-volume distance (shortest distance between the hulls)
- bbd: bounding box disparity (positive continues combination of IoU and v2v)
<br />

- IoU_p: point-based intersection over union of an underlying pointcloud
- pd: distance between the centers of the box
- dd: difference in dimensions
- od_e: orientation difference using angular difference of euler angles
- od_R: orientation difference using rotationmatrices

### The metrics are available as:
- stand-alone functions
- open3d extension
- ROS-node

The Code will be published very soon!
