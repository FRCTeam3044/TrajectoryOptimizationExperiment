

# jormungander safe utility functions
# vectors are all length 2, rotations are all in form [cos(theta), sin(theta)]

import math

def get_index(wptInd, sgmtInd, samples ):
    return sum(samples[:wptInd])+sgmtInd
def rotateBy(vec, rot):
    return (vec[0]*rot[0] - vec[1]*rot[1], vec[0]*rot[1] + vec[1]*rot[0])
def cross(a, b):
    return a[0]*b[1] - a[1]*b[0]
def negRot(rot):
    return (rot[0], -rot[1])
def sqrNorm(vec):
    return vec[0]*vec[0] + vec[1]*vec[1]

def fac(n):
    prod = 1
    for i in range(n):
        prod *= i+1
    return prod
def pow(x,n):
    prod = 1
    for i in range(n):
        prod*=x
    return prod

# don't use these, use jormungander.autodiff.cos/sin instead
def taylorCos(x):
    s = 0
    for i in range(10):
        p = 2*i
        pos = pow(-1,i)
        s += pos*pow(x,p)/fac(p)
    return s
def taylorSin(x):
    s = 0
    for i in range(10):
        p = 2*i + 1
        pos = pow(-1,i)
        s += pos*pow(x,p)/fac(p)
    return s

def rotAdd(a,b):
    return (a[0]*b[0] - a[1]*b[1], a[0]*b[1] + a[1]*b[0])
def rotSub(a,b):
    return rotAdd(a, negRot(b))
def sqrDist(a, b):
    return (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1])

def angleWrap(ang):
    return (ang + math.pi) % (2 * math.pi) - math.pi