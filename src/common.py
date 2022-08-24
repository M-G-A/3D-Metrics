import open3d as o3d
import numpy as np


default_colors = np.array([ [0,0,0],
                            [255,255,255],
                            [0,101,189],
                            [0,82,147],
                            [100,160,200],
                            [152,198,234],
                            [153,153,153],
                            [227,114,34],
                            [162,173,0],
                            [218,215,203],
                            [0,90,90],
                            [90,90,0]    ]) 

def available_methodes():
    return ['bbd',
            'v2v',
            'IoU_v',
            'IoU_p',
            'pd',
            'dd',
            'od_e',
            'od_R'
            ]

def positionDifference(pos1,pos2):
    return np.linalg.norm(pos1-pos2)

def dimensionDifference(dim1,dim2):
    return abs(dim1.prod()-dim2.prod())  # one could also use l2 norm instead of volume
    
def circDifference_euler(rot1,rot2,circ=360):
    from scipy.spatial.transform import Rotation as R
    ang1 = R.from_matrix(rot1).as_euler('xyz')
    ang2 = R.from_matrix(rot2).as_euler('xyz')
    return sum(abs(np.mod(ang1-ang2 + circ/2, circ) - circ/2)) # one could also use l2 norm instead of l1

def rotDifference(R1,R2):
    return np.arccos((np.trace(R1@R2.T)-1)/2) # transpose of a rot-matrix is its inverse

def computeDist(a,b):
    n=np.zeros([len(a),len(b)])
    for i,p in enumerate(a):
        diff = b-p
        diff = np.linalg.norm(diff,axis=1)
        n[i,:] = diff
    return n

def edge2edgeDist(lines1,lines2, check=True, eps = 1e-8):
    m1 = (lines1[:,1]-lines1[:,0]).T
    m2 = (lines2[:,1]-lines2[:,0]).T

    #m1 = m1# / np.linalg.norm(m1,axis=0)
    #m2 = m2# / np.linalg.norm(m2,axis=0)
    e = (lines1[:,0] - lines2[:,0]).T

    det = np.diag(- (m1.T@m1) * (m2.T@m2) + (m1.T@m2)**2) # = valid
    with np.errstate(divide='ignore',invalid='ignore'):
        t2 = np.diag((-m1.T@m1) * (m2.T@e) + (m1.T@e) * (m1.T@m2)) / det 
        t1 = np.diag((m2.T@m2) * (m1.T@e) - (m2.T@e) * (m1.T@m2)) / det
    
    P1 = lines1[:,0].T + m1 * t1
    P2 = lines2[:,0].T + m2 * t2
    
    valid = (det!=0)
    valid = valid & (0-eps<=t1) & (t1<=1+eps) & (0-eps<=t2) & (t2<=1+eps) if check else valid
    P1 = P1[:,valid]
    P2 = P2[:,valid]
    
    dist = np.linalg.norm(P1-P2,axis=0)

    return list(zip(dist,P1.T,P2.T))

def line2pointDist(line,points,check=True,eps = 1e-8):
    m = line[1] - line[0]
    t = (points - line[0])@m / (m@m)
    P = line[0] + (np.expand_dims(m, axis=1) * t).T

    if check:
        valid = (0-eps<=t) & (t<=1+eps)
        P = P[valid]
        points = points[valid]

    dist = np.linalg.norm(P-points,axis=1)

    return list(zip(dist,P,points))
    
class plane():
    def __init__(self,p0,n):
        self.p0 = p0
        self.n = n/np.linalg.norm(n)
        
    def __init__(self,p0,p1,p2):
        self.p0 = p0
        self.p1 = p1
        self.p2 = p2
        self.v = np.stack([p1-p0,p2-p0]).T
        n = np.cross(self.v[:,0],self.v[:,1])
        self.n = n/np.linalg.norm(n)
    
    def intersect_lines(self, lines, eps=1e-8):
        """Finds intersections with lines given by tuples"""
        m = lines[:,1]-lines[:,0] # gradient
        dot = self.n @ m.T # angles between plane and lines

        valid = abs(dot) > eps # only non parallel lines have valid solutions

        t = (self.n @ (self.p0 - lines[valid,0]).T) / dot[valid] # line parameter
        intersections = lines[valid,0] + (m[valid]*t[..., np.newaxis])
        
        return intersections, t
    
    def project_points(self, points, check=True, eps = 1e-8):
        v = (points-self.p0).T
        dist = self.n @ v
        prj_points = points - self.n * dist[..., np.newaxis]
        
        if check:
            t = np.linalg.inv(self.v.T @ self.v) @ self.v.T @ v
            valid = (0-eps<=t[0]) & (t[0]<=1+eps) & (0-eps<=t[1]) & (t[1]<=1+eps)
            points = points[valid]
            prj_points = prj_points[valid]
            dist = dist[valid]
        return list(zip(abs(dist),points,prj_points))

### visualization functions
def plot_hull(hull, c=[1,1,1], ax=None):
    import matplotlib.pyplot as plt
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
    pts = hull.points
    for s in hull.simplices:
        s = np.append(s, s[0])  # Here we cycle back to the first coordinate
        ax.plot(pts[s, 0], pts[s, 1], pts[s, 2], "-", color=c, marker = '.',linewidth=1.5)

def plot_bb(T,c,ax=None):
    import matplotlib.pyplot as plt
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-2,4)
        ax.set_ylim(-2,4)
        ax.set_zlim(-2,4)
    cube = np.array([  [-0.5, -0.5, -0.5, 1],
                       [ 0.5, -0.5, -0.5, 1],
                       [-0.5,  0.5, -0.5, 1],
                       [-0.5, -0.5,  0.5, 1],
                       [ 0.5,  0.5,  0.5, 1],
                       [-0.5,  0.5,  0.5, 1],
                       [ 0.5, -0.5,  0.5, 1],
                       [ 0.5,  0.5, -0.5, 1] ]).T  # 3D location of unit cube
    
    vertices = [[0, 1], [1, 7], [2, 7], [0, 2], [3, 6], [6, 4], 
                [5, 4], [3, 5], [0, 3], [1, 6], [7, 4], [2, 5]] # vertices of a cube
    planes = [0,6]
    faces = [[0,2],[4,6],[0,4],[2,6],[1,5],[3,7]]
    
    points = T @ cube
    
    for f in faces:
        XYZ = np.array([points[:3,vertices[f[0]]],points[:3,vertices[f[1]]]])
        ax.plot_surface(XYZ[:,0,:],XYZ[:,1,:],XYZ[:,2,:], color=list(c)+[0.1], linewidth=0.5, edgecolor=list(c)+[0.8])
        ax.scatter(points[0],points[1],points[2],color=c)
