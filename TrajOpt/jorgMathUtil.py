

# jormungander safe utility functions
# vectors are all length 2, rotations are all in form [cos(theta), sin(theta)]

import math
from jormungandr import autodiff

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
def sub(a,b):
    return (a[0] - b[0], a[1] - b[1])
def add(a,b):
    return (a[0] + b[0], a[1] + b[1])
def neg(a):
    return (-a[0], -a[1])
def scale(a, t):
    return (a[0]*t, a[1]*t)
def dot(a,b):
    return a[0]*b[0] + a[1]*b[1]
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
    return sqrNorm(sub(a,b))
def lerp(a,b,t):
    return add(a, scale(sub(b,a),t))
def angleWrap(ang):
    return (ang + math.pi) % (2 * math.pi) - math.pi

def max(a,b):
    return 0.5 * (1 + autodiff.sign(b - a)) * (b - a) + a
def min(a,b):
    return -0.5 * (1 + autodiff.sign(b - a)) * (b - a) + b
def pointLineDist(p,l1,l2):
    l = sub(l2,l1)
    v = sub(p, l1)
    t = dot(v,l)/sqrNorm(l)
    tBound = max(min(t,1),0)
    i = lerp(l1, l2, tBound)
    return sqrNorm(sub(i, p))