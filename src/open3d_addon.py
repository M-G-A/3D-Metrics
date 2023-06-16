import numpy as np
import open3d as o3d
from common import *

def compare(bbs, metric='BBD'):
    n = len(bbs)
    d = np.zeros(n)
    for i in range(n):
        for j in range(i,n):
            d[i,j] = getattr(bbs[i],metric)(bb[j])
    return d
    
class OrientedBoundingBox(o3d.geometry.OrientedBoundingBox):
    def get_box_faces(self):
        cor = np.asarray(self.get_box_points())
        faces = [[0, 1, 2], [0, 1, 3], [0, 2, 3], [4, 5, 6], [4, 5, 7], [4, 6, 7]] # faces of a cube
        
        face_array = []
        for p0,p1,p2 in faces:
            f = plane(cor[p0],cor[p1],cor[p2])
            face_array.append(f)
        return face_array
    
    def get_box_edges(self):
        lines = o3d.geometry.LineSet.create_from_oriented_bounding_box(self)
        edges = []
        for i in range(12):
            edges.append(lines.get_line_coordinate(i))
        return np.array(edges)
    
    def intersect_lines(self,lines, check=True, eps=1e-10):
        poi = np.empty([1,3])
        for face in self.get_box_faces():
            inters,t = face.intersect_lines(lines, eps)
            poi = np.concatenate([poi,inters])
        
        if check:
            poi_temp = o3d.utility.Vector3dVector(poi)
            self.extent = self.extent + eps
            valid = self.get_point_indices_within_bounding_box(poi_temp)
            self.extent = self.extent - eps
            poi = poi[valid]
        return o3d.utility.Vector3dVector(poi)
            
    def bbd(self,box2, ax=None):
        """Computes the bounding box disparity with another box"""
        if ax is not None:
            plot_bb(self.getT(),c = default_colors[2]/255, ax=ax)
            plot_bb(box2.getT(),c = default_colors[8]/255, ax=ax)
            
        d = 1-self.IoU_v(box2, ax=ax)
        if d == 1:
            d = self.v2v(box2, ax=ax)+1
        return d   
        
    def v2v(self, box2, ax=None, eps = 1e-10):
        """Computes the volume-to-volume distance to another box, which is defined by the shortest path between both hulls"""
        """ note if there is an intersection v2v can give an erroneous value, because it doesn't check for face-face interactions. This is on purpose, because bbd already checks for this """
        poi = []
        f1 = self.get_box_faces()
        f2 = box2.get_box_faces()
        cor1 = np.asarray(self.get_box_points())
        cor2 = np.asarray(box2.get_box_points())
        l1 = self.get_box_edges()
        l2 = box2.get_box_edges()
        
        [ poi.extend(f.project_points(cor2,eps = eps)) for f in f1 ]
        [ poi.extend(f.project_points(cor1,eps = eps)) for f in f2 ]
        
        [ poi.extend(edge2edgeDist(l1,np.expand_dims(l, axis=0),eps = eps)) for l in l2 ] # https://math.stackexchange.com/questions/2213165/find-shortest-distance-between-lines-in-3d
        
        [ poi.extend(line2pointDist(l,cor2,eps = eps)) for l in l1 ]
        [ poi.extend(line2pointDist(l,cor1,eps = eps)) for l in l2 ]
        
        n = computeDist(cor1,cor2)
        r,c = np.unravel_index(n.argmin(), n.shape)
        poi.append((n.min(),cor1[r],cor2[c]))
        poi.sort(key=lambda tup: tup[0]) 
        
        if ax is not None:
            ax.plot([poi[0][1][0],poi[0][2][0]],[poi[0][1][1],poi[0][2][1]],[poi[0][1][2],poi[0][2][2]],"-", color=default_colors[7]/255, marker = '.',linewidth=1.5)
        
        return poi[0][0]
    
    def IoU_p(self, box2, pc):
        """Computes the intersection over union of the points inside of two bounding boxes"""
        # get points inside the boxes
        indices1 = self.get_point_indices_within_bounding_box(pc.points)
        indices2 = box2.get_point_indices_within_bounding_box(pc.points)
        
        # sum number of points inside the boxes
        intersection = len(set(indices1) & set(indices2))
        union = len(set(list(indices1)+list(indices2)))
        IoU = float(intersection)/union
        return IoU
    
    def getT(self):
        """ Combines position, rotation, and dimension to a transformation matrix """
        T = np.vstack([np.column_stack([self.R*self.extent,self.center]),[0,0,0,1]])
        return T
    
    def IoU_v(self, box2, eps = 1e-10, ax=None):
        """Computes the intersection over union of the volume of the two bounding boxes"""
        from scipy.spatial import ConvexHull
        poi = []
        # add corners to pois
        poi = self.get_box_points()
        poi.extend(box2.get_box_points())

        # add face-edge intersections of box1 (self)
        edges = box2.get_box_edges()
        poi.extend(self.intersect_lines(edges, False))
        # add face-edge intersections of box2
        edges = self.get_box_edges()
        poi.extend(box2.intersect_lines(edges, False))
        
        # check if poi are valid (inside each box)
        self.extent = self.extent + eps # only used for possible float precission errors - theoretically should be set to 0
        box2.extent = box2.extent + eps
        valid = list(set(self.get_point_indices_within_bounding_box(poi)) &
                     set(box2.get_point_indices_within_bounding_box(poi)))
        poi = np.asarray(poi)
        #print(np.unique(np.around(poi[valid],decimals=8), axis=0))
        
        self.extent = self.extent - eps
        box2.extent = box2.extent - eps
        
        # try to create convex hull. If it's not possible the boxes don't intersect.
        # TODO: switch to open3d ConvexHull as soon as calculating the volume is implemented
        try:
            h = ConvexHull(poi[valid])
            if ax is not None:
                plot_hull(h,c=default_colors[7]/255,ax=ax)
        except:
            return 0
        
        # calculate volumes
        intersection = h.volume
        union = self.volume() + box2.volume() - intersection
        IoU = intersection/union
        #print(self.volume(),box2.volume(), intersection)
        return IoU
        
    def pd(self, box2):
        """ returns the distance the centers of the boxes """
        return positionDifference(self.center,box2.center)

    def dd(self, box2):
        """ returns the volume differences of the boxes """
        return dimensionDifference(self.extent,box2.extent)
    
    def od_e(self, box2):
        """ returns the sum of absolut circular differences of the boxes euler angles """
        return circDifference_euler(self.R, box2.R)
    	
    def od_R(self, box2):
        """ returns the differences in orientation, based on the rotation matrices of the boxes"""
        return rotDifference(self.R, box2.R)
