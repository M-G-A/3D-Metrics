#!/usr/bin/env python
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy as rpy
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray

from metrics import compare, available_methodes

def rosDataCallback(marker_msg, args):
    bbs = []
    for marker in marker_msg:
        pos = np.array([ marker.pose.position.x,
                         marker.pose.position.y,
                         marker.pose.position.z ])
        rot = np.array([ marker.pose.orientation.x,
                         marker.pose.orientation.y,
                         marker.pose.orientation.z,
                         marker.pose.orientation.w ])
        dim = np.array([ marker.scale.x,
                         marker.scale.y,
                         marker.scale.z ])
        r = R.from_quat(rot).as_matrix()
        T = np.vstack([np.column_stack([dim*r,pos]),[0,0,0,1]])
        bbs.append(T)
    res = compare(bbs,args[1],pc)

def main(metric='bbd', pc_path=None):
    rospy.init_node('3Dmetrics')
    pub = rospy.Publisher('metrics',Float64)
    rospy.Subscriber('bounding_boxes', MarkerArray, rosDataCallback, (pub,metric))
    
    print("ROS initialized. Node is ready!")
    rpy.spin()
    
if __name__ == '__main__':
    argv = rospy.myargv(argv=sys.argv)
    if len(argv) < 2:
        print("using BBD as metric")
        main()
    else:
        assert (argv[1] in available_methodes()), 'the metric ' + argv[1] 'is not yet implemented'
        print("using "+ argv[1] +" as metric")
        if metric = "IoU_p":
            assert (len(argv) > 2), 'point cloud path is missing'
            import open3d as o3d
            pc = o3d.io.read_point_cloud(argv[2])
        main(metric = argv[1])
