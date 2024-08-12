import warnings
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import matplotlib
import math
import numpy as np
#https://matplotlib.org/stable/gallery/lines_bars_and_markers/multicolored_line.html
def colored_line(x, y, c, ax, **lc_kwargs):
    """
    Plot a line with a color specified along the line by a third value.

    It does this by creating a collection of line segments. Each line segment is
    made up of two straight lines each connecting the current (x, y) point to the
    midpoints of the lines connecting the current point with its two neighbors.
    This creates a smooth line with no gaps between the line segments.

    Parameters
    ----------
    x, y : array-like
        The horizontal and vertical coordinates of the data points.
    c : array-like
        The color values, which should be the same size as x and y.
    ax : Axes
        Axis object on which to plot the colored line.
    **lc_kwargs
        Any additional arguments to pass to matplotlib.collections.LineCollection
        constructor. This should not include the array keyword argument because
        that is set to the color argument. If provided, it will be overridden.

    Returns
    -------
    matplotlib.collections.LineCollection
        The generated line collection representing the colored line.
    """
    if "array" in lc_kwargs:
        warnings.warn('The provided "array" keyword argument will be overridden')

    # Default the capstyle to butt so that the line segments smoothly line up
    default_kwargs = {"capstyle": "butt"}
    default_kwargs.update(lc_kwargs)

    # Compute the midpoints of the line segments. Include the first and last points
    # twice so we don't need any special syntax later to handle them.
    x = np.asarray(x)
    y = np.asarray(y)
    x_midpts = np.hstack((x[0], 0.5 * (x[1:] + x[:-1]), x[-1]))
    y_midpts = np.hstack((y[0], 0.5 * (y[1:] + y[:-1]), y[-1]))

    # Determine the start, middle, and end coordinate pair of each line segment.
    # Use the reshape to add an extra dimension so each pair of points is in its
    # own list. Then concatenate them to create:
    # [
    #   [(x1_start, y1_start), (x1_mid, y1_mid), (x1_end, y1_end)],
    #   [(x2_start, y2_start), (x2_mid, y2_mid), (x2_end, y2_end)],
    #   ...
    # ]
    coord_start = np.column_stack((x_midpts[:-1], y_midpts[:-1]))[:, np.newaxis, :]
    coord_mid = np.column_stack((x, y))[:, np.newaxis, :]
    coord_end = np.column_stack((x_midpts[1:], y_midpts[1:]))[:, np.newaxis, :]
    segments = np.concatenate((coord_start, coord_mid, coord_end), axis=1)

    lc = LineCollection(segments, **default_kwargs)
    lc.set_array(c)  # set the colors of each segment

    return ax.add_collection(lc)

def _plot_robot_trajectory(axis,fig, x, y, v):
    lines = colored_line(x,y,v, axis)
    fig.colorbar(lines, label="velocity (m/s)")
    

def _plot_robot_rotation(axis, x, y, theta, botSize):
    for xn,yn,t in zip(x,y,theta):
        rect = matplotlib.patches.Rectangle((xn-botSize/2, yn-botSize/2), botSize, botSize, angle=math.degrees(t), rotation_point='center',facecolor='None', edgecolor='black')
        axis.add_patch(rect)

def _plot_obs(axis, obs):
    for o in obs:
        o.plot(axis)
    
def _plot_wpts(axis, wpts):
    for wpt in wpts:
        wpt.plot(axis)

def plot_solution(probDef):
    assert probDef.solution.solved, "problem must be solved to plot solution"
    fig1, axis = plt.subplots()
    # axis = plt.gca()

    _plot_obs(axis, probDef.obs)

    _plot_robot_rotation(axis, probDef.solution.x, probDef.solution.y, probDef.solution.theta, probDef.botParams.botEdgeSize)

    _plot_robot_trajectory(axis, fig1, probDef.solution.x, probDef.solution.y, probDef.solution.v)

    _plot_wpts(axis, probDef.wpts )
    plt.axis('equal')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.show()

def plot_initialization(probDef):
    assert not probDef.solution.solved, "can't plot initialization after problem is solved"
    fig1, axis = plt.subplots()
    _plot_obs(axis, probDef.obs)
    dts_matched = probDef.solution.getDtsMatched(probDef,[dt.value() for dt in probDef.solution.dts])
    theta = probDef.solution.calculateTheta(probDef, [o.value() for o in probDef.solution.omega], dts_matched)
    _plot_robot_rotation(axis, [x.value() for x in probDef.solution.x], [y.value() for y in probDef.solution.y], theta, probDef.botParams.botEdgeSize)

    vx = [v.value() for v in probDef.solution.vx]
    vy = [v.value() for v in probDef.solution.vy]

    _plot_robot_trajectory(axis, fig1, [x.value() for x in probDef.solution.x], [y.value() for y in probDef.solution.y], probDef.solution.calculateV(vx,vy))
    _plot_wpts(axis, probDef.wpts )
    plt.axis('equal')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.show()

def plot_value_graph(probDef, key):
    plt.plot(probDef.solution.time, getattr(probDef.solution, key))
    for wpt in probDef.wpts:
        if key in wpt.waypoint_dict:
            plt.plot(wpt.time, wpt.waypoint_dict[key], 'bo')
    
    plt.title(key)
    plt.show()

