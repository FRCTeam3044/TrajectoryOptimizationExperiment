
from trajOpt.jorgMathUtil import angleWrap, sqrNorm, sqrDist
from trajOpt.swerveOpt import build_problem
from trajOpt.constraints import InitialWaypoint

from jormungandr.optimization import OptimizationProblem

import numpy as np
from dataclasses import dataclass
import time
import math
class SwerveSolution:
    """
    This class stores the decision variables for the optimization problem, and will extract and compute the implicit and explicit values in the solution for use after solving. 
    """
    def __init__(self, probDef) -> None:
        self.solved = False

        sampTot = sum(probDef.samples)
        sgmtCnt = len(probDef.samples)
        problem = probDef.problem
        # x position of robot
        self.x = [problem.decision_variable() for _ in range(sampTot)]
        # y position of robot
        self.y = [problem.decision_variable() for _ in range(sampTot)]
        # x velocity of robot
        self.vx = [problem.decision_variable() for _ in range(sampTot)]
        # y velocity of robot
        self.vy = [problem.decision_variable() for _ in range(sampTot)]

        # self.vxa = [problem.decision_variable() for _ in range(sampTot)]
        # self.vya = [problem.decision_variable() for _ in range(sampTot)]
        # self.tcos = [problem.decision_variable() for _ in range(sampTot)]
        # self.tsin = [problem.decision_variable() for _ in range(sampTot)]
        # angular velocity of robot
        self.omega = [problem.decision_variable() for _ in range(sampTot)]
        # x acceleration of robot
        self.ax = [problem.decision_variable() for _ in range(sampTot)]
        # y acceleration of robot
        self.ay = [problem.decision_variable() for _ in range(sampTot)]
        # angular acceleration of robot
        self.alpha = [problem.decision_variable() for _ in range(sampTot)]

        # Fy = [[problem.decision_variable() for _ in range(moduleCnt)] for _ in range(sampTot)]
        # Fx = [[problem.decision_variable() for _ in range(moduleCnt)] for _ in range(sampTot)]
        
        # dt for each segment in the problem. dt is the time between each sample. Will be uniform within each segment.
        self.dts = [problem.decision_variable() for _ in range(sgmtCnt)]
        self.dts_matched = self.getDtsMatched(probDef, self.dts)

    def finalize_solution(self,probDef):
        """
        Extracts solved values after the problem completes and computes implicit values like total time and robot angle.
        """
        self.x = [n.value() for n in self.x]
        self.y = [n.value() for n in self.y]
        self.vx = [n.value() for n in self.vx]
        self.vy = [n.value() for n in self.vy]
        self.omega = [n.value() for n in self.omega]
        self.ax = [n.value() for n in self.ax]
        self.ay = [n.value() for n in self.ay]
        self.alpha = [n.value() for n in self.alpha]
        self.dts = [n.value() for n in self.dts]

        self.dt_seg = self.dts
        dts_matched = self.getDtsMatched(probDef, self.dts)
        self.dts = dts_matched

        self.time = [float(t) for t in np.cumsum(dts_matched)]
        self.end_time = self.time[-1]
        self.samples = probDef.samples
        self.wpts = probDef.wpts
        self.v = self.calculateV(self.vx, self.vy)
        self.a = [math.sqrt(sqrNorm((x,y))) for x,y in zip(self.ax, self.ay)]
        
        self.solved = True

        self.theta = self.calculateTheta(probDef, self.omega, self.dts)
        for wpt in probDef.wpts:
            wpt.setSolutionTime(self.time[wpt.firstInd])

    def calculateTheta(self,probDef, omega, dts_matched):
        theta = []
        for omegan, dt in zip(omega, dts_matched):
            if theta == []:
                theta.append(probDef.wpts[0].theta + omegan*dt)
            else:
                theta.append(theta[-1] + omegan*dt)
        return theta
        
    def getDtsMatched(self,probDef, dts):
        if self.solved:
            return self.dts
        
        dts_matched = []
        for dt_seg in [[dt]*Ns for dt, Ns in zip(dts, probDef.samples)]:
            dts_matched.extend(dt_seg)
        return dts_matched
    
    def calculateV(self, vx, vy):
        return [math.sqrt(sqrNorm((x,y))) for x, y in zip(vx, vy)]
    
    def __repr__(self) -> str:
        if self.solved:
            out = f"""Solved Solution:
            Trajectory Total Time: {self.end_time}
            dt per segment: {self.dt_seg}
            """
        else:
            out = f"""Unsolved solution with {len(self.x)} total samples"""
        return out
@dataclass
class BotParams():
    """
    Dataclass to hold and compute robot properties necessary for the problem.
    """
    jerkLim: float = 1 #m/s^3
    accelLim: float = 3 #m/s^2
    omegaLim: float = 8 #rad/s
    alphaLim: float = 15 #rad/s^2

    # could also be max robot speed
    maxWheelVel: float = 4.8768 #m/s

    # assumes square bot with module at each corner, length of one side
    botEdgeSize: float = 1 #m

    @property
    def minWidth(self):
        """additional name for robot edge size. This is the minimum width between any of the module, which for a square is just the length of the edge"""
        return self.botEdgeSize
    
    @property
    def modPos(self):
        """ module positions relative to center of bot"""
        return [(self.botEdgeSize/2, self.botEdgeSize/2),
                (-self.botEdgeSize/2, self.botEdgeSize/2),
                (-self.botEdgeSize/2, -self.botEdgeSize/2),
                (self.botEdgeSize/2, -self.botEdgeSize/2)]

    @property
    def circumcircleRadius(self):
        edge = self.botEdgeSize
        return math.sqrt(edge*edge + edge*edge)/2
class ProblemDefinition():
    """
    This class contains all the different necessary components for our optimization method. Call .build() to create the problem decision variables and setup constraints, and .solve() to run the problem.
    """
    def __init__(self,botParams, wpts, obs):
        # assert issubclass(type(wpts[0]), InitialWaypoint), f"First waypoint must be initial waypoint, current type is: {type(wpts[0])}"
        self.problem = OptimizationProblem()
        
        self.wpts = wpts
        self.obs = obs
        self.botParams = botParams
        self.samples = self.get_samples(wpts)

        self.solved = False
        self.solution = SwerveSolution(self)

        totSamp = 0
        for sgmtInd, (wpt, samp)  in enumerate(zip(wpts[:-1], self.samples)):
            wpt.setInds(totSamp, totSamp+samp, sgmtInd)
            totSamp += samp
        wpts[-1].setInds(totSamp-1, totSamp-1, len(wpts)-1)
        
        prev = None
        for wpt1, wpt2 in zip(wpts[:-1], wpts[1:]):
            wpt1.setNeighbors(prev, wpt2)
            prev = wpt1
        
        wpts[-1].setNeighbors(prev, None)
    def solve(self, **kwargs):
        a = time.time()
        self.problem.solve(**kwargs)
        b = time.time()
        print(f"Problem Solve Time: " + str(b-a))
        self.solution.finalize_solution(self)
        self.solved = True

    def build(self):
        a = time.time()
        build_problem(self)
        b = time.time()
        print(f"Problem Build Time: " + str(b-a))

    def __repr__(self) -> str:
        out = ""
        out += f"""{"Unsolved Problem Definition: " if not self.solved else "Solved Problem Definition: "}
        {repr(self.botParams)}
        {repr(self.wpts)}
        {repr(self.obs)}
        Samples: {repr(self.samples)}
        {repr(self.solution)}
        """
        return out

    def get_samples(self, wpts, sampPerMeter=5, sampPerRad=15, minSamp=10):
        """
        Utility function to decide how many samples per segment should be added to the problem.

        sampPerMeter: will add this many samples per meter to the segment based on the distance between adjacent waypoints
        sampPerRad: will add this many samples per radian to the segment based on the radial distance between adjacent waypoints
        minSamp: the minimum number of samples a segment can have

        Angular and translational waypoints are calculated separatly and the maximum value is used.
        """
        samples = []
        assert wpts[0].theta is not None, "You should use an initial waypoint at the front of the waypoint list"
        theta = wpts[0].theta
        for last_wpt, next_wpt in zip(wpts[:-1], wpts[1:]):
            a = (last_wpt.x, last_wpt.y)
            b = (next_wpt.x, next_wpt.y)
            dist = math.sqrt(sqrDist(a,b))
            samp = int(dist)*sampPerMeter
            if next_wpt.hasValue('theta'):
                angSamp = int(abs(angleWrap(theta-next_wpt.theta))*sampPerRad)
                samp = max(samp, angSamp)
                theta=next_wpt.theta

            samp = max(samp, minSamp)
            samples.append(samp)

        return samples