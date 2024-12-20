from trajOpt.jorgMathUtil import get_index, rotateBy, cross, negRot, sqrNorm, fac, pow, taylorCos, taylorSin, rotAdd, rotSub, angleWrap

import math


# def build_problem(wpts, samples, obs=[], approx=False):
def build_problem(probDef):
    """Calls all necessary steps to build a complete problem given a properly created problem definition object."""
    kinematicConstraints(probDef)
    dynamicConstraints(probDef)

        
    applyExternalConstraints(probDef)
    applyInitialGuess(probDef)

    setObjective(probDef)


def setObjective(probDef):
    T_tot = 0
    for wptInd, (Ns, dt) in enumerate(zip(probDef.samples, probDef.solution.dts)):

        T_seg = dt*Ns
        T_tot += T_seg
    
    probDef.problem.minimize(T_tot)

def kinematicConstraints(probDef):
    last_angle = probDef.wpts[0].theta 

    sweep_sum = 0
    for wptInd, (Ns, dt) in enumerate(zip(probDef.samples, probDef.solution.dts)):
        #https://github.com/SleipnirGroup/Choreo/blob/main/trajoptlib/src/SwerveTrajectoryGenerator.cpp#L103
        #ensures that the robot does not travel more than it's width in a given sample interval
        #prevents teleporting through obstacles as defined in trajoptlib
        # probDef.problem.subject_to(dt*maxWheelVel <= probDef.botParams.circumcircleRadius*2)



        #constrain dt to be positive, and give initial value
        probDef.problem.subject_to(dt >= 0)
 


        for sgmtInd in range(Ns):
            i = get_index(wptInd, sgmtInd, probDef.samples)
            if i == 0:
                continue
            xn = (probDef.solution.x[i], probDef.solution.y[i])
            xn1 = (probDef.solution.x[i-1], probDef.solution.y[i-1])
            # tn = (tcos[i], tsin[i])
            # tn1 = (tcos[i-1], tsin[i-1])
            vn = (probDef.solution.vx[i], probDef.solution.vy[i])
            # vna = (vxa[i], vya[i])
            vn1 = (probDef.solution.vx[i-1], probDef.solution.vy[i-1])
            omegan = probDef.solution.omega[i]
            omegan1 = probDef.solution.omega[i-1]
            an = (probDef.solution.ax[i], probDef.solution.ay[i])
            an1 = (probDef.solution.ax[i-1], probDef.solution.ay[i-1])
            alphan = probDef.solution.alpha[i]

            sweep_sum += omegan*dt + alphan*0.5*dt*dt
            # tn_tn1 = rotSub(tn, tn1)
            # tn1_p_omega_dt = rotAdd(tn1, (autodiff.cos(omegan*dt), autodiff.sin(omegan*dt)))
            #kinematic constraints
            #https://github.com/SleipnirGroup/Choreo/blob/main/trajoptlib/src/SwerveTrajectoryGenerator.cpp#L146
            for xyi in range(len(xn)):

                # particle acceleration formula. Constrains the position of the robot for each successive sample to align with it's velocity and acceleration
                probDef.problem.subject_to(xn1[xyi] + vn[xyi] * dt + an[xyi] * 0.5 * dt * dt == xn[xyi])

                #angular velocity constraint formulation directly from https://github.com/SleipnirGroup/Choreo/blob/main/trajoptlib/src/SwerveTrajectoryGenerator.cpp#L148
                # problem.subject_to(tn_tn1[xyi] == (autodiff.cos(omegan*dt) if xyi == 0 else autodiff.sin(omegan*dt)))

                #new angle velocity constraint formulation, several order magnitude speedup for some reason
                # problem.subject_to(tn[xyi] == tn1_p_omega_dt[xyi])

                # particle accleration formula, but for velocity
                probDef.problem.subject_to(vn1[xyi] + an[xyi]*dt == vn[xyi])
                # problem.subject_to(autodiff.abs(vn[xyi]) == vna[xyi])

            # rotational acceleration formula
            probDef.problem.subject_to(omegan1 + alphan*dt == omegan)

            # add quantity limits
            # jerk limit
            # probDef.problem.subject_to(sqrNorm((an[0]-an1[0], an[1]-an1[1]))<= probDef.botParams.jerkLim**2)
            # linear acceleration limit
            probDef.problem.subject_to(sqrNorm(an) <= probDef.botParams.accelLim**2)
            # angular velocity limit
            probDef.problem.subject_to(omegan**2 <= probDef.botParams.omegaLim**2)
            # angular acceleration limit, could also add an angular jerk limit if necessary
            probDef.problem.subject_to(alphan**2 <= probDef.botParams.alphaLim**2)

            #failed attempt at "look at" constraint
            # problem.subject_to((sweep_sum+last_angle) == autodiff.tan((-10-xn[1])/ (-10-xn[0])))

        # This ensures that the robot will hit the specific waypoint theta angles. Instead of having the optimizer solve for the angle of the robot at each step directly (which is expensive), 
        # we constrain the distance travelled based on the angular velocity and acceleration between each theta constraining waypoint to be equal to the shortest angular path to the next theta waypoint.
        if probDef.wpts[wptInd+1].hasValue("theta"):
            wpt_theta = probDef.wpts[wptInd+1].theta
            angle_change = wpt_theta - last_angle
            wrapped_angle = angleWrap(angle_change)
            probDef.problem.subject_to(sweep_sum == wrapped_angle )
            sweep_sum = 0
            last_angle = wpt_theta
        
def dynamicConstraints(probDef):
    for i in range(sum(probDef.samples)):
        
        vn = (probDef.solution.vx[i], probDef.solution.vy[i])
        omegan = probDef.solution.omega[i]
        # Fxn = Fx[i]
        # Fyn = Fy[i]
        # FxNet = sum(Fxn)
        # FyNet = sum(Fyn)

        # tauNet = 0
        # for mod in range(moduleCnt):
            # trans = modPos[mod]
            # F = (Fx[i][mod], Fy[i][mod])
            #manual rotation matrix
            #https://github.com/SleipnirGroup/Choreo/blob/main/trajoptlib/include/trajopt/geometry/Translation2.hpp#L140
            # r = rotateBy(trans,tn)
            #manual cross product
            #https://github.com/SleipnirGroup/Choreo/blob/main/trajoptlib/include/trajopt/geometry/Translation2.hpp#L172
            # tauNet += cross(r,F)

        # vWrtBot = rotateBy(vn, negRot(tn))
        maxR = 0
        #use worst case rotation limits
        for mod in range(len(probDef.botParams.modPos)):
            trans = probDef.botParams.modPos[mod]
            # vwheelWrtBot = (vWrtBot[0] - trans[1]*omegan, vWrtBot[1] + trans[0]*omegan)
            #kinematic wheel constaint on velocity
            # problem.subject_to(sqrNorm(vwheelWrtBot) <= maxWheelVel**2)

            #dynamic wheel constraint on produced force
            # problem.subject_to(sqrNorm((Fxn[mod], Fyn[mod])) <= maxWheelForce**2)

            r = math.sqrt(sqrNorm(trans))**2
            if r > maxR:
                maxR = r
        # kinematic worst case for wheel velocity
        #this is wrong, but gets close to the correct behavior
        # for some reason jorg has an awful time calculating v using autodiff.sqrt as opposed to v^2 (or the abs value of vx/vy using autodiff.abs) 
        # this is as close as I could get to a correct formulation for now, although it would be better for this to become a conservative bounding function 
        vWheelWorstCase = sqrNorm(vn) + r*r*omegan*omegan# + |2*v*r*omegan|
        probDef.problem.subject_to(vWheelWorstCase<= probDef.botParams.maxWheelVel**2)

        # dynamic constraints on force/accel
        # problem.subject_to(FxNet == mass*ax[i])
        # problem.subject_to(FyNet == mass*ay[i])
        # problem.subject_to(tauNet == moi*alpha[i])

def applyExternalConstraints(probDef):
    # apply waypoint constraints
    for wpt in probDef.wpts:
        
        #constrain waypoints
        wpt.apply(probDef)



    # apply obstacle constraints
    for o in probDef.obs:
        o.apply(probDef, 0, sum(probDef.samples))
        
def applyInitialGuess( probDef):
    """ Create the initial guess values for the problem. This linearly interpolates the values of each sample before constrained waypoints for each value type.
    Applying an initial guess like this significantly boosts the speed and robustness of solving the problem. """
    solution = probDef.solution
    wpts = probDef.wpts
    samples = probDef.samples
    thetaPerPoint = getInitialThetaPerPoint(probDef)
    for wptInd in range(len(wpts[:-1])):
        i = get_index(wptInd, 0, samples)
        #constrain waypoints
        #warm starting, kinda
        Ns = samples[wptInd]
        solution.dts[wptInd].set_value(5/Ns)

        for sgmtInd,frac in enumerate(range(Ns)):
            i = get_index(wptInd, sgmtInd, samples)
            xstart = wpts[wptInd].x + (wpts[wptInd+1].x-wpts[wptInd].x)*(frac/Ns)
            ystart = wpts[wptInd].y + (wpts[wptInd+1].y-wpts[wptInd].y)*(frac/Ns)
            solution.x[i].set_value(xstart)
            solution.y[i].set_value(ystart)

            if i > 0:
                solution.vx[i].set_value((solution.x[i].value()-solution.x[i-1].value())/solution.dts[wptInd].value())
                solution.vy[i].set_value((solution.y[i].value()-solution.y[i-1].value())/solution.dts[wptInd].value())
                solution.ax[i].set_value((solution.vx[i].value()-solution.vx[i-1].value())/solution.dts[wptInd].value())
                solution.ay[i].set_value((solution.vy[i].value()-solution.vy[i-1].value())/solution.dts[wptInd].value())
                solution.omega[i].set_value((thetaPerPoint[i]-thetaPerPoint[i-1])/solution.dts[wptInd].value())
                solution.alpha[i].set_value((solution.omega[i].value()-solution.omega[i-1].value())/solution.dts[wptInd].value())


def getInitialThetaPerPoint(probDef):
    """ Compute an interpolated value for theta based on waypoints. Needed to set initial values of angular velocity."""
    #there is definitely a more elegant way to do this
    wpts = probDef.wpts
    samples = probDef.samples
    thetas = [wpt.theta if wpt.hasValue("theta") else None for wpt in wpts]
    tNextTarg = []
    NAccum = 0
    for Ns, t in zip(samples,thetas[1:]):
        if t is not None:
            tNextTarg.append((NAccum + Ns,t))
            NAccum = 0
        else:
            NAccum += Ns
    
    thetaPerPoint = []
    thetaPrev = wpts[0].theta
    for Ns, t in tNextTarg:
        thetaChange = angleWrap(t - thetaPrev)
        for frac,sgmtInd in enumerate(range(Ns)):
            thetaPerPoint.append(thetaPrev + thetaChange*(frac/Ns))
        
        thetaPrev = t
    
    if len(thetaPerPoint) < sum(samples):
        thetaPerPoint.extend([thetaPerPoint[-1] if len(thetaPerPoint)>0 else thetaPrev]*(sum(samples)-len(thetaPerPoint)))

    return thetaPerPoint
