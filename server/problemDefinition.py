from jorgMathUtil import sqrDist
import math
from jorgMathUtil import angleWrap, sqrNorm

import numpy as np
from dataclasses import dataclass
from constraints import InitialWaypoint
from jormungandr.optimization import OptimizationProblem
from swerveOpt import build_problem
import time
class SwerveSolution:

    def __init__(self, problem, samples) -> None:
        sampTot = sum(samples)
        sgmtCnt = len(samples)
        self.x = [problem.decision_variable() for _ in range(sampTot)]
        self.y = [problem.decision_variable() for _ in range(sampTot)]
        self.vx = [problem.decision_variable() for _ in range(sampTot)]
        self.vy = [problem.decision_variable() for _ in range(sampTot)]
        # self.vxa = [problem.decision_variable() for _ in range(sampTot)]
        # self.vya = [problem.decision_variable() for _ in range(sampTot)]
        # self.tcos = [problem.decision_variable() for _ in range(sampTot)]
        # self.tsin = [problem.decision_variable() for _ in range(sampTot)]
        self.omega = [problem.decision_variable() for _ in range(sampTot)]
        self.ax = [problem.decision_variable() for _ in range(sampTot)]
        self.ay = [problem.decision_variable() for _ in range(sampTot)]
        self.alpha = [problem.decision_variable() for _ in range(sampTot)]

        # Fy = [[problem.decision_variable() for _ in range(moduleCnt)] for _ in range(sampTot)]
        # Fx = [[problem.decision_variable() for _ in range(moduleCnt)] for _ in range(sampTot)]
        
        self.dts = [problem.decision_variable() for _ in range(sgmtCnt)]
        self.solved = False

    def finalize_solution(self,probDef):
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
        dts_matched = []
        for dt_seg in [[dt]*Ns for dt, Ns in zip(self.dts, probDef.samples)]:
            dts_matched.extend(dt_seg)
        self.dts = dts_matched

        self.time = [float(t) for t in np.cumsum(dts_matched)]
        self.end_time = self.time[-1]
        self.samples = probDef.samples
        self.wpts = probDef.wpts
        self.v = [math.sqrt(sqrNorm((x,y))) for x, y in zip(self.vx, self.vy)]
        self.a = [math.sqrt(sqrNorm((x,y))) for x,y in zip(self.ax, self.ay)]
        self.theta = []
        for omegan, dt in zip(self.omega, dts_matched):
            if self.theta == []:
                self.theta.append(probDef.wpts[0].theta + omegan*dt)
            else:
                self.theta.append(self.theta[-1] + omegan*dt)
        self.solved = True
    
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
    
    wheelRad: float # = 0.0381

    jerkLim: float # = 1
    accelLim: float
    omegaLim: float # = 8
    alphaLim: float # = 15

    maxWheelVel: float # = 4.8768
    botEdgeSize: float
    @property
    def wheelMaxAngVel(self):
        return self.maxWheelVel/self.wheelRad
    
    @property
    def minWidth(self):
        return self.botEdgeSize
    
    @property
    def modPos(self):
        return [(self.botEdgeSize/2, self.botEdgeSize/2),
                (-self.botEdgeSize/2, self.botEdgeSize/2),
                (-self.botEdgeSize/2, -self.botEdgeSize/2),
                (self.botEdgeSize/2, -self.botEdgeSize/2)]

class ProblemDefinition():
    def __init__(self,botParams, wpts, obs):
        assert issubclass(type(wpts[0]), InitialWaypoint), "First waypoint must be initial waypoint"
        self.problem = OptimizationProblem()
        
        self.wpts = wpts
        self.obs = obs
        self.botParams = botParams
        self.samples = get_samples(wpts)
        for o in obs:
            o.registerBot(botParams)
        self.solution = SwerveSolution(self.problem, self.samples)
        self.solved = False

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
def get_samples(wpts, sampPerMeter=10, sampPerRad=15, minSamp=10):
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