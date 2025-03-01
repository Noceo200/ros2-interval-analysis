import matplotlib.pyplot as plt
from numpy import mean, pi, cos, sin, sinc, sqrt, tan, arctan, arctan2, tanh, arcsin, arccos, \
    exp, dot, array, log, inf, eye, zeros, ones, inf, size, \
    arange, reshape, vstack, hstack, diag, median, \
    sign, sum, meshgrid, cross, linspace, append, round, trace, rint
from matplotlib.pyplot import *
from matplotlib.cbook import flatten
from numpy.random import randn, rand
from numpy.linalg import inv, det, norm, eig, qr
from scipy.linalg import sqrtm, expm, logm, norm, block_diag

from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from math import factorial
from matplotlib.patches import Ellipse, Rectangle, Circle, Wedge, Polygon, Arc
from matplotlib.collections import PatchCollection


def add1(M):
    M = array(M)
    return vstack((M, ones(M.shape[1])))

def plot2D(ax, M, col='black', w=1):
    ax.plot(M[0, :], M[1, :], col, linewidth=w)

def plot2D_no_ax(M,col='black',w=1):
    plot(M[0, :], M[1, :], col, linewidth = w)    


def tran2H(x, y):
    return array([[1, 0, x], [0, 1, y], [0, 0, 1]])


def rot2H(a):
    return array([[cos(a), -sin(a), 0], [sin(a), cos(a), 0], [0, 0, 1]])

def arrow2H(L):
    e = 0.2
    return add1(L * array([[0, 1, 1 - e, 1, 1 - e], [0, 0, -e, 0, e]]))

def sawtooth(x):
    return (x + pi) % (2 * pi) - pi  # or equivalently   2*arctan(tan(x/2))


def geo_scalar_prod(u,v, θu, θv):
    return u*v*cos(θu-θv)


def dist(a,b):
    xa, ya = a[0:2].flatten()
    xb, yb = b[0:2].flatten()
    return sqrt((xb-xa)**2 + (yb-ya)**2)

def angle(x):
    x=x.flatten()
    return arctan2(x[1],x[0])


def sawtooth(x):
    return (x+pi)%(2*pi)-pi
