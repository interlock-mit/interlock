import math
import numpy as np
import statistics

def geometric(left_fit, right_fit, h):
    '''
    computes the average and standard deviation of the distance between the left line and the 
    intersection of its normal with the right line; if both are within a predefined range, 
    the geometric test passes
    :param left_fit: a polynomial for the left line
    :param right_fit: a polynomial for the right line
    :param h: the maximum x value used to compare the lines
    '''
    # ax**2 + bx + c
    la, lb, lc = left_fit
    ra, rb, rc = right_fit
    x_coords = np.arange(0, h - 1.0)
    leftpoints = la * (x_coords ** 2.0) + lb * x_coords + lc
    rightpoints = ra * (x_coords ** 2.0) + rb * x_coords + rc

    # check which point from right lane corresponds to the normal line created by left lane points
    lnormalslope = -1/(2.0*la*x_coords+lb)
    diffs = []
    for xcoord,ycoord, m in zip(x_coords, leftpoints, lnormalslope):
        b = ycoord - m*xcoord # slope intercept for normal line

        # solving mx + b = ax**2 + bx + c gives 0 = ax**2 + x*(b - m) + (c - b)
        newa = ra
        newb = rb-m
        newc = rc-b

        # solve quadratic equation and use the closest solution
        xsoln1 = int((-newb + math.sqrt(newb**2.0 - 4*newa*newc)) / (2.0 * newa))
        xsoln2 = int((-newb - math.sqrt(newb**2.0 - 4*newa*newc)) / (2.0 * newa))
        ysoln1 = ra*xsoln1**2.0 + rb*xsoln1 + rc
        ysoln2 = ra*xsoln2**2.0 + rb*xsoln2 + rc
        firstdiff = math.sqrt((xcoord - xsoln1)**2.0 + (ycoord - ysoln1)**2.0)
        seconddiff = math.sqrt((xcoord - xsoln2)**2.0 + (ycoord - ysoln2)**2.0)
        diff = min(firstdiff, seconddiff)
        diffs.append(diff)

    comparednum = len(diffs)
    stdev = statistics.stdev(diffs)
    mean = statistics.mean(diffs)
    return stdev < 1.0 and mean > 3.0 and mean < 6.0
