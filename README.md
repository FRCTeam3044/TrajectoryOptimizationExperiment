# TrajectoryOptimizationExperiment

This is an experimental project started by David to see if realtime trajectory optimization is a feasible goal for the team to pursue. Most of the initial optimization problem definition for the work here is based on [TrajOptLib](https://github.com/SleipnirGroup/Choreo/tree/main/trajoptlib), which is the optimization wrapper for the popular auto software [Choreo](https://github.com/SleipnirGroup/Choreo/tree/main). I used the [Jormungander](https://github.com/SleipnirGroup/Sleipnir/tree/main/jormungandr) optimization frontend for [Slepnir](https://github.com/SleipnirGroup/Sleipnir) to handle the optimization in a similar manner to TrajOptLib. There are still lines in the code that represent constraints used in the trajoptlib formulation. I left them in and commented out to show the difference in formulation. Can be removed once we fully commit to this formulation.

The primary goal of this work is to determine whether it is technically feasible to run a trajectory optimizer in realtime or near-realtime on top of Oxplorer generated trajectories. This repository only contains code for a basic version of problem definition, solving, and visualization, and therefore does none of the engineering required for integration or application. Students may use this as a starting point for learning about trajectory optimization and can build off of this if they wish to develop it into a full solution. I do not intend to contribute more of a working solution to this repository, but will likely continue work on the problem privately in order to better understand potential pit falls and help provide accurate guidance should a student pick up the project in the future. 

Please feel free to ask me any questions you have about this work.

# Install Instructions
If you do not already have and IDE for python installed, I'd recommend following normal FRC setup instructions to get VSCode [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).

If you are already familiar with python development, feel free to use whatever package management system you wish. Otherwise, [I would recommend using anaconda. Install instructions for anaconda.](https://docs.anaconda.com/anaconda/install/)

After installing anaconda, follow the instructions [here](https://conda.io/projects/conda/en/latest/user-guide/getting-started.html) to create an environment for the project. Make sure to activate the environment before moving on.

Once your environment is active, run: 

```
conda install pip
```

Navigate to the top level directory of this repository and run:
```
pip install -r requirements.txt
python -m ipykernel install --user --name=trajOpt
```

You can now open the example.ipynb notebook in VSCode, select trajOpt as the kernel in the upper right, and run the example notebook.


# Example
![Example Trajectory](https://github.com/FRCTeam3044/TrajectoryOptimizationExperiment/blob/main/images/exampleTraj.png)


The above image is the solution for an example problem. I'm using this problem and others like this to profile the behavior of the optimization process in question. This problem was created with the following scenario in mind: The robot starting from rest near an objective located near (2,~5) (think shelf in 2023 or note slide in 2024) should approach that object straight on, come to a full stop in front of the objective, then reach a waypoint at (4,0) while developing speed in the x direction. This could be analogous to a situation like picking up an object from the field, grabbing something from an intake station, scoring a game piece, etc. In order to correctly solve this problem, the robot (starting at (0,0)) must make a quarter turn before driving foward to the objective ((2,3) -> (2,4)), then continue on it's way by reaching the point (4,0) with a velocity of 2 m/s. It must do all this while avoiding the obstacle in red.

The successive black squares indicate the robot's current rotation along it's path. The multicolored line is the center of the robot along it's trajectory and the colors indicate it's speed at that point as indicated by the colorbar to the right. The blue dots are the waypoints for the problem:
1. (0,0): Initial robot position starting from rest
2. (2,3): Robot must reach this point after executing a quarter turn to face the objective, while having no velocity in the x direction in order to approach the objective head-on
3. (2,4): Come to rest in front of the objective
4. (4,0): Reach this point on the way to the next objective with a speed of 2m/s in the x direction.

The total time for this trajectory following reasonable robot characteristics similar to our 2024 robot was 6.325s, with a total solve time of 0.082s on an 11th Gen core i7-11700F @ 2.50GHz. This has not been tested for speed on a viable coprocessor yet, and is the primary remaining variable to whether or not this technique would be feasible for realtime use.

# Simplifying Assumptions
I started this work using the same problem formulation as TrajOptLib, however it seemed unlikely that their problem definition could be made to run in realtime. I have introduced several simplifying assumptions in order to reduce computational burden to replace some of the more complex constraints in their formulation:
1. Instead of explicity constraining maximum force output of the modules, I have the user input a maximum acceleration/jerk/angular acceleration the trajectory should adhere to under the assumption that the robot will be able to produce at least the acceleration/jerk/angular acceleration of the limit provided. [Implementation](https://github.com/FRCTeam3044/TrajectoryOptimizationExperiment/blob/main/TrajOpt/swerveOpt.py#L80)
2. Instead of explicity constraining the maximum velocity of each modules independently, I constrain the velocity/angular velocity such that the the worst case (fastest) a module could be moving given a particular velocity/angular velocity pair in under the maximum velocity for any particular module. [Implementation](https://github.com/FRCTeam3044/TrajectoryOptimizationExperiment/blob/main/TrajOpt/swerveOpt.py#L137)
3. The robot's current rotation in the trajectory is not computed directly in the optimization. Instead we assume that the robot will rotate the shortest distance toward it's next rotation target, and that the integral of it's angular velocity during that time is such that it will reach it's target. [Implementation](https://github.com/FRCTeam3044/TrajectoryOptimizationExperiment/blob/main/TrajOpt/swerveOpt.py#L92)

# Known Bugs
Assumption 2 can accomplish the goal of ensuring all modules stay under their maximum velocity throughout their trajectory, however I found that the optimizer used here has particular trouble computing both the sqrt of the velocity decision variables and the absolute value of the velocity components. With this limitation, I am unable to compute a value for the magnitude of the velocity during the solve step which is required for correctly implementing this constraint. Right now I have ommited the portion of the constraint equation that includes the velocity magnitude term, so the calculated trajectories are liable to violate the maximum speed on a particular module. I will be doing more research into hold to reformulate this constraint to be conservative if an exact constraint cannot be calculated. [Constraint in question.](https://github.com/FRCTeam3044/TrajectoryOptimizationExperiment/blob/main/TrajOpt/swerveOpt.py#L134)

