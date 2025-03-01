from interval_analysis_boat_simu.calcul_tools import *

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


def init_figure(xmin, xmax, ymin, ymax, width=10, height=10, id="fig_id"):
    # fig = figure(figsize=(width, height))
    # ax = fig.add_subplot(111, aspect='equal')
    fig, ax = plt.subplots(num=id)
    plt.suptitle('Simulation', size='x-large')

    ax.xmin = xmin
    ax.xmax = xmax
    ax.ymin = ymin
    ax.ymax = ymax
    clear(ax)
    return fig, ax


def init_table2(data, legend, xmin, xmax, ymin, ymax, text_size=11, length=4, width=2):
    fig, ax = plt.subplots()
    plt.suptitle('Active rules of the sea', size='x-large')
    table = plt.table(cellText=data, loc='center')
    legend_table = plt.table(cellText=legend, loc='bottom')

    table.auto_set_font_size(False)
    table.set_fontsize(text_size)

    # Colouring of specific boxes
    cell_colors = []
    for row in range(len(data)):
        current_row = []
        for col in range(len(data[row])):
            if (row, col) == (0, 0):  # condition to color the box
                current_row.append('lightgray')
            else:
                current_row.append('white')
        cell_colors.append(current_row)

    for row in range(len(data)):
        for col in range(len(data[row])):
            table[row, col].set_facecolor(cell_colors[row][col])

    ax.xmin = xmin
    ax.xmax = xmax
    ax.ymin = ymin
    ax.ymax = ymax
    clear(ax)
    ax.axis('off')
    fig.tight_layout()
    return fig, ax, table

def init_table(rules, legend,fig,ax, text_size=11, length=4, width=2):
    #fig, ax = plt.subplots()
    # fig, ax = plt.subplots(figsize=(3, 0.5))
    plt.suptitle('Active rules of the sea', size='x-large')
    table = plt.table(cellText=rules, loc='center')
    legend_table = plt.table(cellText=legend, loc='bottom')

    # # Modification du style de la première colonne
    # first_column_cells = [table.get_celld()[row, 0] for row in range(len(data))]
    # for cell in first_column_cells:
    #     cell.set_width(length * 1.2)  # Double la largeur de la première colonne

    table.auto_set_font_size(False)
    table.set_fontsize(text_size)
    # table.scale(length, width)
    # table.scale(0.3, 0.3)
    # ax.xmin = xmin
    # ax.xmax = xmax
    # ax.ymin = ymin
    # ax.ymax = ymax
    # Colouring of specific boxes
    cell_colors = []
    for row in range(len(rules)):
        current_row = []
        for col in range(len(rules[row])):
            if (row, col) == (row, 0):  # Condition pour les cases à colorer
                current_row.append('lightgray')
            else:
                current_row.append('white')
        cell_colors.append(current_row)

    for row in range(len(rules)):
        for col in range(len(rules[row])):
            table[row, col].set_facecolor(cell_colors[row][col])

    ax.axis('off')
    fig.tight_layout()
    # plt.subplots_adjust(left=0.7, top=0.3)

    return table

def clear(ax):
    pause(0.001)
    ax.cla()
    ax.set_xlim(ax.xmin, ax.xmax)
    ax.set_ylim(ax.ymin, ax.ymax)


def draw_arrow(ax, x, y, θ, L, col='darkblue', w=1):
    plot2D(ax, tran2H(x, y) @ rot2H(θ) @ arrow2H(L), col, w)


def draw_boat_and_vector(ax, x, col='darkblue', r=0.1, w=2):
    """ Draw a boat with his speed vector """
    mx, my, v, θ = list(x[0:4, 0])
    M = r * array([[-1, 5, 7, 7, 5, -1, -1, -1], [-2, -2, -1, 1, 2, 2, -2, -2]])
    M = add1(M)
    draw_arrow(ax, mx, my, θ, norm(v), 'red')
    plot2D(ax, tran2H(mx, my) @ rot2H(θ) @ M, col, w)


def draw_field_around_c(ax, f, xmin, xmax, ymin, ymax, a, c):
    """ Draw field with the parameter c """
    Mx = arange(xmin, xmax, a)
    My = arange(ymin, ymax, a)
    X1, X2 = meshgrid(Mx, My)
    VX, VY = f(X1, X2, c)
    R = sqrt(VX ** 2 + VY ** 2)
    ax.quiver(Mx, My, VX / R, VY / R)


def draw_field_around_c_new(ax, f, xmin, xmax, ymin, ymax, a, c, D, k, r):
    """ Draw field with the parameter c """
    Mx = arange(xmin, xmax, a)
    My = arange(ymin, ymax, a)
    X1, X2 = meshgrid(Mx, My)
    VX, VY = f(X1, X2, c, D, k, r)
    R = sqrt(VX ** 2 + VY ** 2)
    ax.quiver(Mx, My, VX / R, VY / R)


def draw_circle(ax, center_x, center_y, radius, color):
    circle = plt.Circle((center_x, center_y), radius, fill=False, color=color)
    ax.add_artist(circle)


def draw_disk(ax, c, r, col, alph=0.7, w=1):
    # draw_disk(ax,array([[1],[2]]),0.5,"blue")
    e = Ellipse(xy=c, width=2 * r, height=2 * r, angle=0, linewidth=w)
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(alph)  # transparency
    e.set_facecolor(col)


def draw_arrow_test_arc(x, y, θ, L, col='darkblue', w=1):
    plot2D_no_ax(tran2H(x, y) @ rot2H(θ) @ arrow2H(L), col, w)



def draw_arc(c, a, θ, col, ax):
    s = arange(0, abs(θ), 0.01)
    s = sign(θ) * s
    d = a - c
    r = norm(d)
    alpha = angle(d)
    w = c @ ones((1, size(s))) + r * array([[cos(alpha), -sin(alpha)], [sin(alpha), cos(alpha)]]) @ array(
        [cos(s), sin(s)])
    ax.plot(w[0, :], w[1, :], color=col, linewidth=2)



def draw_cone_arc(x, y, R, sight_angle, cap, ax):
    """
    Args:
        pos_robot (array): coordinates of the robot (x, y)
        R (float): robot detection range
        sight_angle (radian): robot detection angle
        cap (radian): robot heading (angle)
    """
    edge_1_x = x + R * cos(sight_angle / 2 + cap)
    edge_1_y = y + R * sin(sight_angle / 2 + cap)
    edge_2_x = x + R * cos(-sight_angle / 2 + cap)
    edge_2_y = y + R * sin(-sight_angle / 2 + cap)

    edge_2 = array([[edge_2_x], [edge_2_y]])  # Extremity right of the detection zone
    robot = array([[x], [y]])

    # Array to fill for shading the detection zone
    sector_points_x = [x, edge_1_x]
    sector_points_y = [y, edge_1_y]

    # Create the list of points along the arc
    num_arc_points = 200
    arc_angles = np.linspace(cap + sight_angle / 2, cap - sight_angle / 2, num_arc_points)
    arc_x = x + R * cos(arc_angles)
    arc_y = y + R * sin(arc_angles)

    sector_points_x.extend(arc_x)
    sector_points_y.extend(arc_y)

    # Add the other edge of the detection zone to complete the list of sector points
    sector_points_x.append(edge_2_x)
    sector_points_y.append(edge_2_y)

    # Fill the detection zone with color
    ax.fill(sector_points_x, sector_points_y, 'grey', alpha=0.2)

    # Plot the robot's direction lines and position
    #ax.plot([x, edge_1_x], [y, edge_1_y], 'bo', linestyle="-")
    #ax.plot([x, edge_2_x], [y, edge_2_y], 'bo', linestyle="-")
    #ax.plot(x, y, 'ko')

    # Call draw_arc with the specified axis (ax)
    #draw_arc(robot, edge_2, sight_angle, col='grey', ax=ax)
