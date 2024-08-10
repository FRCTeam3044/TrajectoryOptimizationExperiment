import matplotlib.pylab as plt
import math


from dataclasses import dataclass, fields

def constrainTo(problem, state, val):
    for s, v in zip(state, val):
        problem.subject_to(s==v)

class Constraint():

    def apply(self, problem, solution, start_ind, end_ind=None):
        raise NotImplementedError

class Waypoint(Constraint):
    def __init__(self, **waypoint_dict) -> None:
        super().__init__()
        assert 'x' in waypoint_dict.keys()
        assert 'y' in waypoint_dict.keys()

        self.waypoint_dict = waypoint_dict

        for k, v in self.waypoint_dict.items():
            setattr(self, k, v)
        
    def hasValue(self, key):
        return key in self.waypoint_dict.keys()
    
    def apply(self, problem, solution, start_ind):
        prob_vars = [getattr(solution,k)[start_ind] for k in self.waypoint_dict.keys() if k != "theta" ]
        vals = [v for k,v in self.waypoint_dict.items() if k != "theta"]
        constrainTo(problem, prob_vars, vals)

    def __repr__(self) -> str:
        return "Waypoint: " + repr(self.waypoint_dict)
    
    def plot(self, axis):
        axis.plot(self.x, self.y, 'bo')

class FullRestWaypoint(Waypoint):
    def __init__(self, **waypoint_dict) -> None:

        waypoint_dict.update({"vx":0, "vy":0, "ax":0, "ay":0, "omega":0, "alpha":0})
        super().__init__(**waypoint_dict)

class InitialWaypoint(Waypoint):
    def __init__(self, **waypoint_dict) -> None:
        if 'theta' not in waypoint_dict.keys():
            waypoint_dict['theta']=0
        super().__init__(**waypoint_dict)

class FullRestInitialWaypoint(FullRestWaypoint, InitialWaypoint):
    def __init__(self, **waypoint_dict):
        super().__init__( **waypoint_dict)


class Obstacle(Constraint):
    def __init__(self) -> None:
        super().__init__()
    
    def plot(self, axis):
        raise NotImplementedError
    
    def registerBot(self, botParams):
        raise NotImplementedError

class CircularObstacle(Obstacle):

    def __init__(self, x, y, r) -> None:

        self.x, self.y, self.r = x, y ,r
        
        super().__init__()
    
    def plot(self, axis):
        axis.add_patch(plt.Circle((self.x, self.y), self.r,color='r'))

    def registerBot(self, botParams):
        edge = botParams.botEdgeSize
        self.botRad = self.botRad = math.sqrt(edge*edge + edge*edge)/2

    def apply(self, problem,solution, start_ind, end_ind=None):
        x = solution.x[start_ind: end_ind]
        y = solution.y[start_ind: end_ind]
        if end_ind == None:
            end_ind = start_ind+1

        for xn, yn in zip(x,y):    
            problem.subject_to((xn-self.x)*(xn-self.x) + (yn-self.y)*(yn-self.y) >= (self.r+self.botRad)* (self.r+self.botRad))


    def __repr__(self) -> str:
        return f"CircularObstacle(x = {self.x}, y = {self.y}, r = {self.r})"