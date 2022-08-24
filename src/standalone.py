import numpy as np
from common import *

def compare(bbs, metric='BBD'):
    n = len(bbs)
    d = np.zeros(n)
    for i in range(n):
        for j in range(i,n):
            d[i,j] = getattr(bbs[i],metric)(bb[j])
    return n

class OBB():
    def __init__(self,T):
        self.corners = np.array([ [-0.5, -0.5, -0.5, 1],
                            [ 0.5, -0.5, -0.5, 1],
                            [-0.5,  0.5, -0.5, 1],
                            [-0.5, -0.5,  0.5, 1],
                            [ 0.5,  0.5,  0.5, 1],
                            [-0.5,  0.5,  0.5, 1],
                            [ 0.5, -0.5,  0.5, 1],
                            [ 0.5,  0.5, -0.5, 1] ]).T
        self.edges = [[0, 1], [1, 7], [7, 2], [2, 0], [3, 6], [6, 4], 
                        [4, 5], [5, 3], [0, 3], [1, 6], [7, 4], [2, 5]] # vertices of a cube
        self.faces = [[0, 1, 2], [0, 1, 3], [0, 2, 3], [4, 5, 6], [4, 5, 7], [4, 6, 7]] # faces of a cube
        self.T = T
    
    def get_box_points(self):
        return (self.T @ self.corners)[:3].T
    
    def get_box_faces(self):
        cor = self.get_box_points() 
        face_array = []
        for p0,p1,p2 in self.faces:
            f = plane(cor[p0],cor[p1],cor[p2])
            face_array.append(f)
        return face_array
    
    def get_box_edges(self):
        cor = self.get_box_points()
        edges = []
        for [e0,e1] in self.edges:
            edges.append([cor[e0],cor[e1]])
        return np.array(edges)
    
    def get_point_indices_within_bounding_box(self, points,eps=1e-10):
        temp = np.linalg.inv(self.T)@np.vstack((points.T,np.ones(len(points))))
        return np.all(temp[:3]<=0.5+eps,0) & np.all(temp[:3]>=-0.5-eps,0)
    
    def intersect_lines(self,lines, check=True, eps=1e-10):
        poi = np.empty([1,3])
        for face in self.get_box_faces():
            inters,t = face.intersect_lines(lines, eps)
            poi = np.concatenate([poi,inters])
        
        if check:
            valid = self.get_point_indices_within_bounding_box(poi)
            poi = poi[valid]
        return poi
            
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
        
        [ poi.extend(edge2edgeDist(l1,np.expand_dims(l, axis=0),eps = 1e-10)) for l in l2 ] # https://math.stackexchange.com/questions/2213165/find-shortest-distance-between-lines-in-3d
        
        [ poi.extend(line2pointDist(l,cor2,eps = eps)) for l in l1 ]
        [ poi.extend(line2pointDist(l,cor1,eps = eps)) for l in l2 ]
        
        n = computeDist(cor1,cor2)
        r,c = np.unravel_index(n.argmin(), n.shape)
        poi.append((n.min(),cor1[r],cor2[c]))
        poi.sort(key=lambda tup: tup[0]) 
        
        if ax is not None:
            ax.plot([poi[0][1][0],poi[0][2][0]],[poi[0][1][1],poi[0][2][1]],[poi[0][1][2],poi[0][2][2]],"-", color=default_colors[7]/255, marker = '.',linewidth=1.5)
        
        return poi[0][0]
    
    def IoU_p(self, box2, pc_points):
        """Computes the intersection over union of the points inside of two bounding boxes"""
        # get points inside the boxes
        indices1 = self.get_point_indices_within_bounding_box(pc_points)
        indices2 = box2.get_point_indices_within_bounding_box(pc_points)
        
        # sum number of points inside the boxes
        intersection = sum(indices1 & indices2)
        union = sum(indices1 | indices2)
        IoU = float(intersection)/union
        return IoU
    
    def getT(self):
        return self.T
    
    def volume(self):
        p,r,d = self.get_prd()
        return np.prod(d)
        
    def get_prd(self):
        """ splits the transformation matrix in position, rotation, and dimension"""
        p = self.T[:3,3]
        r = self.T[:3,:3]
        d = np.linalg.norm(r,axis=0)
        r = r/d
        return p,r,d
    
    def IoU_v(self, box2, eps = 1e-10, ax=None):
        """Computes the intersection over union of the volume of the two bounding boxes"""
        from scipy.spatial import ConvexHull
        # add corners to pois
        poi = self.get_box_points()
        poi = np.vstack((poi, box2.get_box_points()))

        # add face-edge intersections of box1 (self)
        edges = box2.get_box_edges()
        poi = np.vstack((poi,self.intersect_lines(edges, False)))
        
        # add face-edge intersections of box2
        edges = self.get_box_edges()
        poi = np.vstack((poi,box2.intersect_lines(edges, False)))
        
        valid = (self.get_point_indices_within_bounding_box(poi, eps=eps) &
                  box2.get_point_indices_within_bounding_box(poi,eps=eps))
        
        # try to create convex hull. If it's not possible the boxes don't intersect.
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
        return IoU

    def pd(self, box2):
        """ returns the distance the centers of the boxes """
        p,r,d = self.get_prd()
        p2,r2,d2 = box2.get_prd()
        return positionDifference(p,p2)
    
    def dd(self, box2):
        """ returns the volume differences of the boxes """
        p,r,d = self.get_prd()
        p2,r2,d2 = box2.get_prd()
        return dimensionDifference(d,d2)
    
    def od_e(self, box2):
        """ returns the sum of absolut circular differences of the boxes euler angles """
        p,r,d = self.get_prd()
        p2,r2,d2 = box2.get_prd()
        return circDifference_euler(r, r2)
    
    def od_R(self, box2):
        """ returns the differences in orientation, based on the rotation matrices of the boxes"""
        p,r,d = self.get_prd()
        p2,r2,d2 = box2.get_prd()
        return rotDifference(r, r2)