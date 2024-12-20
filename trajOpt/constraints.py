import matplotlib.pylab as plt
import math
from dataclasses import dataclass, fields
from trajOpt.jorgMathUtil import pointLineDist, sub, sqrNorm,sqrDist, pointLineDist, dot
from jormungandr import autodiff
def constrainTo(problem, state, val):
    for s, v in zip(state, val):
        problem.subject_to(s==v)

class Constraint():

    # start_ind is inclusive, end_ind is exclusive
    def apply(self, probDef, start_ind, end_ind=None):
        raise NotImplementedError
    
    def plot(self, axis):
        pass

class BoundValue(Constraint):
    def __init__(self, key, min=None, max=None) -> None:
        super().__init__()
        self.key = key
        self.min = min
        self.max = max

    def apply(self, probDef, start_ind, end_ind=None):
        val = getattr(probDef.solution,self.key)[start_ind: end_ind]
        for v in val:
            if self.min is not None:
                probDef.problem.subject_to(v >= self.min)

            if self.max is not None:
                probDef.problem.subject_to(v <= self.max)      

class BindValue(Constraint):
    def __init__(self, key, value) -> None:
        super().__init__()
        self.key = key
        self.value = value

    def apply(self, probDef, start_ind, end_ind=None):
        val = getattr(probDef.solution,self.key)[start_ind: end_ind]
        for v in val:
            probDef.problem.subject_to(v == self.value)

class BoundVelocityConstraint(Constraint):
    def __init__(self, min=None, max=None) -> None:
        super().__init__()
        self.min = min
        self.max = max
    def apply(self, probDef, start_ind, end_ind=None):
        vx = probDef.solution.vx[start_ind: end_ind]
        vy = probDef.solution.vy[start_ind: end_ind]
        for vxn, vyn in zip(vx, vy):
            if self.min is not None:
                probDef.problem.subject_to(sqrNorm((vxn, vyn)) >= self.min**2)

            if self.max is not None:
                probDef.problem.subject_to(sqrNorm((vxn, vyn)) <= self.max**2) 

class TakesAtLeast(Constraint):
    def __init__(self, time) -> None:
        self.time = time
    def apply(self, probDef, start_ind, end_ind=None):
        assert end_ind - start_ind > 1, "takes at least should only be applied to entire segments or trajectories"
        dts = probDef.solution.dts_matched[start_ind: end_ind]
        probDef.problem.subject_to(sum(dts) >= self.time)
        


class Near(Constraint):
    def __init__(self, pt, tolMeters) -> None:
        super().__init__()
        self.pt = pt
        self.tolMeters = tolMeters

    def apply(self, probDef, start_ind, end_ind=None):
        x = probDef.solution.x[start_ind: end_ind]
        y = probDef.solution.y[start_ind: end_ind]
        for xn, yn in zip(x,y):
            probDef.problem.subject_to(sqrDist(self.pt,(xn,yn)) <= self.tolMeters**2)

class OnLine(Constraint):
    # you cannot use multiples of this constraint at one time
    def __init__(self, pts) -> None:
        super().__init__()
        self.pts = pts
    
    def apply(self, probDef, start_ind, end_ind=None):
        x = probDef.solution.x[start_ind: end_ind]
        y = probDef.solution.y[start_ind: end_ind]
        for xn, yn in zip(x,y):
            probDef.problem.subject_to(pointLineDist((xn, yn), self.pts[0], self.pts[1]) == 0)
    
class OnLines(Constraint):
    # works, but is pretty slow, use only if necessary
    # cannot use multiples of this at the same time, but this method could be rewritten to allow for all arbitrary line segments to be expressed in just one constraint instead of a set of continuous segments
    def __init__(self, pts) -> None:
        super().__init__()
        self.pts = pts
    
    def apply(self, probDef, start_ind, end_ind=None):
        self.slacks = [probDef.problem.decision_variable() for _ in range(len(self.pts)-1)]
        
        x = probDef.solution.x[start_ind: end_ind]
        y = probDef.solution.y[start_ind: end_ind]
        for xn, yn in zip(x,y):
            for l1, l2, slack in zip(self.pts[:-1], self.pts[1:], self.slacks):
                probDef.problem.subject_to(slack >= 0)
                probDef.problem.subject_to(slack*pointLineDist((xn, yn), l1, l2) == 0)
                
        probDef.problem.subject_to(sum(self.slacks) > 0.1)

class ConvexPolygonBound(Constraint):
    def __init__(self, pts) -> None:
        super().__init__()
        assert len(pts) > 2
        self.pts = pts

    def apply(self, probDef, start_ind, end_ind=None):
        x = probDef.solution.x[start_ind: end_ind]
        y = probDef.solution.y[start_ind: end_ind]

        rad = probDef.botParams.circumcircleRadius
        for l1, l2 in zip(self.pts[:-1], self.pts[1:]):
            for xn, yn in zip(x,y):   
                hyp_vec = sub(l2,l1) 
                hyp_norm = (hyp_vec[1],-hyp_vec[0])
                hyp_offset = dot(hyp_norm, l1)
                p = (xn,yn)
                probDef.problem.subject_to(dot(hyp_norm,p)-hyp_offset >= 0)
        
        # connect last and first waypoint to close the shape
        if len(self.pts) > 2:
            l1 = self.pts[-1]
            l2 = self.pts[0]
            for xn, yn in zip(x,y):    
                hyp_vec = sub(l2,l1) 
                hyp_norm = (hyp_vec[1],-hyp_vec[0])
                hyp_offset = dot(hyp_norm, l1)
                p = (xn,yn)
                probDef.problem.subject_to(dot(hyp_norm,p)-hyp_offset >= 0)          
    def plot(self, axis):
        x = [p[0] for p in self.pts] + [self.pts[0][0]]
        y = [p[1] for p in self.pts] + [self.pts[0][1]]
        axis.plot(x,y, 'g')

class LookAt(Constraint):
    def __init__(self, pt, tolRad, localOffset=0) -> None:
        super().__init__()
        self.pt = pt
        self.tolRad = tolRad
        self.localOffset = localOffset

    def apply(self, probDef, start_ind, end_ind=None):
        omega = probDef.solution.omega
        alpha = probDef.solution.alpha
        dts_matched= probDef.solution.dts_matched
        prevTheta = 0
        initialWpt = None
        initialInd = 0
        for wpt in probDef.wpts:
            if wpt.lastIndPlusOne > start_ind+1:
                break
            if wpt.hasValue("theta") and not isinstance(wpt, UnboundWaypoint):
                prevTheta = wpt.theta
                initialInd = wpt.firstInd
                initialWpt = wpt
        thetaBegin = prevTheta
        for i in range(initialInd, start_ind ):
            thetaBegin += omega[i]*dts_matched[i] + alpha[i]*0.5*dts_matched[i]*dts_matched[i]
        thetaCur = thetaBegin
        for i in range(start_ind, end_ind):
            # haven't entirely thought through why this works, copied from trajoptlib here:
            # https://github.com/SleipnirGroup/Choreo/blob/main/trajoptlib/include/trajopt/constraint/PointAtConstraint.hpp#L59C1-L63C79
            thetaCur +=  omega[i]*dts_matched[i] + alpha[i]*0.5*dts_matched[i]*dts_matched[i]
            pose = (probDef.solution.x[i], probDef.solution.y[i])
            dxdy = sub(self.pt, pose)
            dot = autodiff.cos(thetaCur-self.localOffset)*dxdy[0] + autodiff.sin(thetaCur-self.localOffset)*dxdy[1]
            probDef.problem.subject_to(dot >= autodiff.cos(self.tolRad)*autodiff.hypot(dxdy[0], dxdy[1]))

class Obstacle(Constraint):
    def __init__(self) -> None:
        super().__init__()
    
    def plot(self, axis):
        raise NotImplementedError

class CircularObstacle(Obstacle):

    def __init__(self, x, y, r) -> None:

        self.x, self.y, self.r = x, y ,r
        
        super().__init__()
    
    def plot(self, axis):
        axis.add_patch(plt.Circle((self.x, self.y), self.r,color='r'))

    def apply(self, probDef, start_ind, end_ind=None):
        x = probDef.solution.x[start_ind: end_ind]
        y = probDef.solution.y[start_ind: end_ind]

        rad = probDef.botParams.circumcircleRadius
        for xn, yn in zip(x,y):    
            probDef.problem.subject_to((xn-self.x)*(xn-self.x) + (yn-self.y)*(yn-self.y) >= (self.r+rad)* (self.r+rad))


    def __repr__(self) -> str:
        return f"CircularObstacle(x = {self.x}, y = {self.y}, r = {self.r})"
    
class LineObstacle(Obstacle):

    def __init__(self, pts) -> None:
        super().__init__()
        assert len(pts) > 1
        self.pts = pts
    def apply(self, probDef, start_ind, end_ind=None):
        x = probDef.solution.x[start_ind: end_ind]
        y = probDef.solution.y[start_ind: end_ind]

        rad = probDef.botParams.circumcircleRadius
        for l1, l2 in zip(self.pts[:-1], self.pts[1:]):
            for xn, yn in zip(x,y):    
                p = (xn,yn)
                probDef.problem.subject_to(pointLineDist(p, l1, l2)**2 >= rad**2)
        
        # connect last and first waypoint to close the shape
        if len(self.pts) > 2:
            l1 = self.pts[-1]
            l2 = self.pts[0]
            for xn, yn in zip(x,y):    
                p = (xn,yn)
                probDef.problem.subject_to(pointLineDist(p, l1, l2)**2 >= rad**2)            

    def plot(self, axis):
        x = [p[0] for p in self.pts] + [self.pts[0][0]]
        y = [p[1] for p in self.pts] + [self.pts[0][1]]
        axis.plot(x,y, 'r')

class Waypoint():
    def __init__(self, sgmtConsts=[], ptConsts=[], **waypoint_dict) -> None:
        super().__init__()
        assert 'x' in waypoint_dict.keys()
        assert 'y' in waypoint_dict.keys()
        self.ptConsts = ptConsts
        self.sgmtConsts=sgmtConsts
        self.waypoint_dict = waypoint_dict
        self.prev = None
        self.next = None
        for k, v in self.waypoint_dict.items():
            setattr(self, k, v)
        
    def hasValue(self, key):
        return key in self.waypoint_dict.keys()
    def setNeighbors(self, prev, next):
        self.prev = prev
        self.next = next
    
    def hasPrev(self):
        return self.prev is not None
    def hasNext(self):
        return self.next is not None
    def apply(self, probDef):
        self.applyWaypointDictConstraints(probDef)
        self.applySegmentConstraints(probDef)
        self.applyPointConstraints(probDef)
    
    def applyPointConstraints(self, probDef):
        for con in self.ptConsts:
            con.apply(probDef, self.firstInd, self.firstInd+1)

    def applySegmentConstraints(self, probDef):
        for con in self.sgmtConsts:
            con.apply(probDef, self.firstInd, self.lastIndPlusOne)

    def applyWaypointDictConstraints(self, probDef):
        prob_vars = [getattr(probDef.solution,k)[self.firstInd] for k in self.waypoint_dict.keys() if k != "theta" ]
        vals = [v for k,v in self.waypoint_dict.items() if k != "theta"]
        constrainTo(probDef.problem, prob_vars, vals)

    def setInds(self, firstInd, lastIndPlusOne, segmentInd):
        self.firstInd = firstInd
        self.lastIndPlusOne = lastIndPlusOne
        self.segmentInd = segmentInd

    def setSolutionTime(self, time):
        self.time = time

    def __repr__(self) -> str:
        return "Waypoint: " + repr(self.waypoint_dict)
    
    def plot(self, axis):
        axis.plot(self.x, self.y, 'bo')

class FullRestWaypoint(Waypoint):
    def __init__(self,sgmtConsts=[], ptConsts=[], **waypoint_dict) -> None:

        waypoint_dict.update({"vx":0, "vy":0, "ax":0, "ay":0, "omega":0, "alpha":0})
        super().__init__(sgmtConsts, ptConsts,**waypoint_dict)

class InitialWaypoint(Waypoint):
    def __init__(self,sgmtConsts=[], ptConsts=[], **waypoint_dict) -> None:
        if 'theta' not in waypoint_dict.keys():
            waypoint_dict['theta']=0
        super().__init__(sgmtConsts, ptConsts,**waypoint_dict)

class FullRestInitialWaypoint(FullRestWaypoint, InitialWaypoint):
    def __init__(self, sgmtConsts=[], ptConsts=[],**waypoint_dict):
        super().__init__( sgmtConsts,ptConsts, **waypoint_dict)

class UnboundWaypoint(Waypoint):
    # this is bad, and will be redone once waypoints use constraints for binding vars instead of waypoint_dict
    def __init__(self,sgmtConsts=[], ptConsts=[], equalizeDt=True, **waypoint_dict) -> None:
        super().__init__(sgmtConsts,ptConsts,**waypoint_dict)
        self.equalizeDt = equalizeDt

    def apply(self, probDef):
        self.applySegmentConstraints(probDef)
        self.applyPointConstraints(probDef)
        if self.equalizeDt:
            # do nothing if this happens to be last waypoint, it already shares the dt value with the last segment
            if self.segmentInd >= len(probDef.solution.dts):
                return
            probDef.problem.subject_to(probDef.solution.dts[self.segmentInd] == probDef.solution.dts[self.segmentInd-1])

    def plot(self, axis):
        axis.plot(self.x, self.y, 'go')
