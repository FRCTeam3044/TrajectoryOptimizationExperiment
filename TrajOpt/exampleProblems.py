
from trajOpt.constraints import CircularObstacle,  FullRestInitialWaypoint, FullRestWaypoint, Waypoint, LineObstacle, UnboundWaypoint, BoundValue, BindValue, LookAt, BoundVelocityConstraint, Near
from trajOpt.problemDefinition import  BotParams, ProblemDefinition
import math
from trajOpt.jorgMathUtil import lerp, sqrDist

def defaultBot():
    return BotParams(wheelRad=0.0381,
            jerkLim=2,
            accelLim=6,
            omegaLim=8,
            alphaLim=15,
            maxWheelVel=4.8768,
            botEdgeSize=1)

def _driveStraightWpts():
    return (FullRestInitialWaypoint(x=0, y=0, theta=0),
            FullRestWaypoint(x=5, y=0, theta=0))


######### Full Problems #########
def driveStraightProblem():
    wpts = _driveStraightWpts()
    botParams = defaultBot()
    return ProblemDefinition(botParams, wpts, [])   
 
def driveStraightAroundObsProblemBroken():
    #intentionally left broken to test initialization vs line obs
    wpts = _driveStraightWpts()
    botParams = defaultBot()
    obs = [LineObstacle([(1,1),(3,-1),(4,1)])]
    return ProblemDefinition(botParams, wpts, obs)

def driveStraightAroundObsSpecialInitProblem():
    wpts = (FullRestInitialWaypoint(x=0, y=0, theta=0),
            UnboundWaypoint(x=3,y=-2),
            FullRestWaypoint(x=5, y=0, theta=0))
    botParams = defaultBot()
    obs = [LineObstacle([(1,1),(3,-1),(4,1)])]
    return ProblemDefinition(botParams, wpts, obs)

def pickupStationProblem():
    wpts = (FullRestInitialWaypoint(x=0, y=0, theta=0),
        Waypoint(x=2, y=3, vx=0, theta=math.pi/2),
        FullRestWaypoint(x=2, y=4, theta=math.pi/2),
        Waypoint(x=5, y=0, vx=2, vy=0, theta=0))

    botParams = defaultBot()

    obs = [CircularObstacle(x=2,y=1,r=.5)]
    # obs = []
    return ProblemDefinition(botParams, wpts, obs)

def pickupStationProblemApproachLimit():
    wpts = (FullRestInitialWaypoint(x=0, y=0, theta=0),
        Waypoint(x=2, y=3, vx=0, theta=math.pi/2,
                 sgmtConsts=[BoundValue('vy', max=1), BindValue('vx', 0)]),
        FullRestWaypoint(x=2, y=4, theta=math.pi/2),
        Waypoint(x=5, y=0, vx=2, vy=0, theta=0))

    botParams = defaultBot()

    obs = [CircularObstacle(x=2,y=1,r=.5)]
    # obs = []
    return ProblemDefinition(botParams, wpts, obs)

def pickupNoteAndShoot():
            #start facing speaker at (0,-1)
    startPos =(0,0)
    speakerPos =(0,-2)
    notePos = (2,4)
    lookTolerance = 0.05
    backOfRobot = math.pi
    maxShotRange = 5
    startLookAt = lerp(startPos, notePos, 0.75)
    t= maxShotRange/math.sqrt(sqrDist(speakerPos,notePos))
    suggestedEnd = lerp(speakerPos, notePos, t)
    
    wpts = (FullRestInitialWaypoint(x=startPos[0], y=startPos[1], theta=-math.pi/2),
            #add waypoint to begin lookAt constraint (looking at the note)
            UnboundWaypoint(x=startLookAt[0], y=startLookAt[1], 
                            sgmtConsts=[LookAt((notePos[0], notePos[1]),lookTolerance, localOffset=backOfRobot)]),
            #pickup the note
            Waypoint(x=notePos[0], y=notePos[1],
                    sgmtConsts=[ BoundValue('omega',max=0.5)]),
            #move back toward speaker and face with low velocity in order to shoot
            UnboundWaypoint(x=suggestedEnd[0], y=suggestedEnd[1],
                     ptConsts=[BoundVelocityConstraint(max=0.5),Near((speakerPos[0], speakerPos[1]),maxShotRange), LookAt((speakerPos[0], speakerPos[1]), lookTolerance), BindValue('omega',0)] ))

    botParams = defaultBot()
    # don't cross center field (problem not actual to scale, just a test)
    obs = [BoundValue('y', max=notePos[1]+0.25)]
    return ProblemDefinition(botParams, wpts, obs)

def pickup2NotesAndShoot():
    # this is getting close to real functionality, prototypes could be defined for each step so that they get built and strung together on the fly
    startPos =(0,0)
    speakerPos =(0,-2)
    notePos1 = (1,4)
    notePos2 = (3,4)
    lookTolerance = 0.05
    backOfRobot = math.pi
    maxShotRange = 5
    startLookAt1 = lerp(startPos, notePos1, 0.75)
    t1= maxShotRange/math.sqrt(sqrDist(speakerPos,notePos1))
    suggestedShot1 = lerp(speakerPos, notePos1, t1)
    
    startLookAt2 = lerp(suggestedShot1, notePos2,0.75)
    t2= maxShotRange/math.sqrt(sqrDist(speakerPos,notePos1))
    suggestedShot2 = lerp(speakerPos, notePos2, t2)

    wpts = (FullRestInitialWaypoint(x=startPos[0], y=startPos[1], theta=-math.pi/2),
            #add waypoint to begin lookAt constraint (looking at the note)
            UnboundWaypoint(x=startLookAt1[0], y=startLookAt1[1], 
                            sgmtConsts=[LookAt((notePos1[0], notePos1[1]),lookTolerance, localOffset=backOfRobot)]),
            #pickup the note
            Waypoint(x=notePos1[0], y=notePos1[1],
                    sgmtConsts=[ BoundValue('omega',max=0.5)]),
            #move back toward speaker and face with low velocity in order to shoot
            UnboundWaypoint(x=suggestedShot1[0], y=suggestedShot1[1],
                     ptConsts=[BoundVelocityConstraint(max=0.5),Near((speakerPos[0], speakerPos[1]),maxShotRange), LookAt((speakerPos[0], speakerPos[1]), lookTolerance), BindValue('omega',0)] ),
            
            #Start to look at note 2
            UnboundWaypoint(x=startLookAt2[0], y=startLookAt2[1], 
                sgmtConsts=[LookAt((notePos2[0], notePos2[1]),lookTolerance, localOffset=backOfRobot)]),
            #pickup note 2
            Waypoint(x=notePos2[0], y=notePos2[1],
                    sgmtConsts=[ BoundValue('omega',max=0.5)]),
            #move back toward speaker and face with low velocity in order to shoot
            UnboundWaypoint(x=suggestedShot2[0], y=suggestedShot2[1],
                     ptConsts=[BoundVelocityConstraint(max=0.5),Near((speakerPos[0], speakerPos[1]),maxShotRange), LookAt((speakerPos[0], speakerPos[1]), lookTolerance), BindValue('omega',0)] ))
            
    botParams = defaultBot()
    # don't cross center field (problem not actual to scale, just a test)
    obs = [BoundValue('y', max=notePos1[1]+0.25), BoundValue('omega',max=1)]
    return ProblemDefinition(botParams, wpts, obs)

