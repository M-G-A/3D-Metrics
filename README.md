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
<br />

## Paper
You can find more information in the paper
https://arxiv.org/abs/2207.03720

```
@inproceedings{ ,
	author = {Adam, Michael G. and  Piccolrovazzi, Martin and Eger, Sebastian and Steinbach, Eckehard},
	title = {Bounding Box Disparity: 3D Metrics for Object Detection with Full Degree of Freedom},
	booktitle = {IEEE ICIP 2022},
	year = {2022},
	address = {Bordeaux, France},
	month = {Oct},
	language = {en},
}
```

Note of the authors (updated):<br />
Googles implementation of IoU (https://github.com/google-research-datasets/Objectron) apparently assumes boxes, which are only on rotation apart. Our implementation does not have this assumption. <br />
Facebooks implementation in PyTorch3D (https://pytorch3d.org/docs/iou3d, Code published end of 2021) seems to be a similar parallel work, however they meshify the boxes and do not state or explain there equations/method (in the corresponding paper IoU isn’t even discussed). We give those explanations in the paper and we think this makes it more easy to follow ;).<br /> 
Nevertheless, v2v and bbd should be completely new.

## Acknowledgement
This work is funded by Germany’s Federal Ministry of Education and Research within the project KIMaps (grant ID #01IS20031C).
