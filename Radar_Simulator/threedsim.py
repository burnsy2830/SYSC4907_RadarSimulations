"""
@Author: Liam Burns 
@Date: 2024/06/20

This simulates the 3d math behind bistatic radar systems
"""

import random as rand
import math as m
import numpy as np
import scipy
from scipy.optimize import minimize
import sympy as sp
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import fsolve


def threedeecli(tx,rx_1,rx_2,rx_3,drone):

    option = input("Enter Simulation Method h for TDOA , E for range sum, m for more options")
    if option == "H":
        select = input("N for numerically, M for mathematically NOTE: the hyperbola solution is off currently and the graphimg feature does not work properly")
        if select == "M":
            calculate_position_mathematically_hyperboloid(tx, rx_1, rx_2,rx_3, drone)
        if select == "N":
            calculate_position_mathematically_hyperboloid(tx, rx_1, rx_2,rx_3, drone)

    if option == "E":
        select = input("N for numerically, M for mathematically")
        if select == "M":
             calculate_position_mathematically_ellipsoid(tx, rx_1, rx_2,rx_3, drone)
        if select == "N":
            calculate_position_numericaly_ellipsoid(tx, rx_1, rx_2,rx_3, drone)
    
def calculate_position_numericaly_ellipsoid(tx,rx_1,rx_2,rx_3,drone):
    a1, b1, c1, h1, k1, l1, a2, b2, c2, h2, k2, l2, a3, b3, c3, h3, k3, l3 = calculate_ellipsoid_params(tx, rx_1, rx_2, rx_3, drone, False)
    elips_params = [(a1,b1,c1,h1,k1,l1),
                    (a2,b2,c2,h2,k2,l2),
                    (a3,b3,c3,h3,k3,l3)]
    sol = find_intersection(elips_params)

    solution = [float(sol[0]),float(sol[1]),float(sol[2])]
    print("\n SOLUTIONS: ", solution)
    ans = input("Plot Ellipsoids? (y/n): ")
    if ans == "y":
        plot_ellipsoids(h1, k1, l1, a1, b1, c1, h2, k2, l2, a2, b2, c2, h3, k3, l3, a3, b3, c3, solution,True)

def calculate_position_mathematically_ellipsoid(tx, rx_1, rx_2, rx_3, drone):
    a1, b1, c1, h1, k1, l1, a2, b2, c2, h2, k2, l2, a3, b3, c3, h3, k3, l3 = calculate_ellipsoid_params(tx, rx_1, rx_2, rx_3, drone, False)

    x = sp.Symbol("x", real=True)
    y = sp.Symbol("y", real=True)
    z = sp.Symbol("z", real=True)

    ellipsoid1_eq = (((x - h1)) ** 2 / a1**2) + ((y - k1) ** 2 / b1**2) + ((z - l1) ** 2 / c1**2) - 1
    ellipsoid2_eq = (((x - h2)) ** 2 / a2**2) + ((y - k2) ** 2 / b2**2) + ((z - l2) ** 2 / c2**2) - 1
    ellipsoid3_eq = (((x - h3)) ** 2 / a3**2) + ((y - k3) ** 2 / b3**2) + ((z - l3) ** 2 / c3**2) - 1

    solution = sp.solve((ellipsoid1_eq, ellipsoid2_eq, ellipsoid3_eq), (x, y, z))
    print("\nSolution: ", solution)

    ans = input("Plot Ellipsoids? (y/n): ")
    if ans == "y":
        plot_ellipsoids(h1, k1, l1, a1, b1, c1, h2, k2, l2, a2, b2, c2, h3, k3, l3, a3, b3, c3, solution,False)

def calculate_position_mathematically_hyperboloid(tx, rx_1, rx_2, rx_3, drone):
    #This very rarley returns a real answer. This is beacuse 
    a1, b1, c1, h1, k1, l1, a2, b2, c2, h2, k2, l2, a3, b3, c3, h3, k3, l3 = calculate_hyperboloid_params(tx, rx_1, rx_2, rx_3, drone, False)
    
    x, y, z = sp.symbols("x y z",real=True)
    
    # Define equations correctly using sp.Eq()
    eq1 = sp.Eq(((x - h1)**2 / a1**2) + ((y - k1)**2 / b1**2) + ((z - l1)**2 / c1**2), 1)
    eq2 = sp.Eq(((x - h2)**2 / a2**2) - ((y - k2)**2 / b2**2) - ((z - l2)**2 / c2**2), 1)
    eq3 = sp.Eq(((x - h3)**2 / a3**2) - ((y - k3)**2 / b3**2) - ((z - l3)**2 / c3**2), 1)
    
    # Solve the system of equations
    solution = sp.solve([eq1, eq2, eq3], (x, y, z))
    print(solution)

    ans = input("Plot Ellipsoids? (y/n): ")
    if ans == "y":
       plot_hyperboloids_and_ellipsoid(a1, b1, c1, h1, k1, l1, a2, b2, c2, h2, k2, l2, a3, b3, c3, h3, k3, l3,[solution.x[0],solution.x[1],solution.x[2]])

def calculate_position_numericaly_hyperboloid(tx, rx_1, rx_2, rx_3, drone):
    # Obtain hyperboloid parameters
    a1, b1, c1, h1, k1, l1, a2, b2, c2, h2, k2, l2, a3, b3, c3, h3, k3, l3 = calculate_hyperboloid_params(tx, rx_1, rx_2, rx_3, drone, False)

    
    print("\nParameters of Ellipsoid 1:", (a1, b1, c1, h1, k1, l1))
    print("\nParameters of Ellipsoid 2:", (a2, b2, c2, h2, k2, l2))
    print("\nParameters of Ellipsoid 3:", (a3, b3, c3, h3, k3, l3))
    
    def equations(vars):
        x, y, z = vars
        eq1 = (((x - h1)) ** 2 / a1**2) + ((y - k1) ** 2 / b1**2) + ((z - l1) ** 2 / c1**2) - 1
        eq2 = (((x - h2)) ** 2 / a2**2) - ((y - k2) ** 2 / b2**2) - ((z - l2) ** 2 / c2**2) - 1
        eq3 = (((x - h3)) ** 2 / a3**2) - ((y - k3) ** 2 / b3**2) - ((z - l3) ** 2 / c3**2) - 1
        return [eq1, eq2, eq3]

    initial_guess = [0.0, 0.0, 0.0]
    solution = scipy.optimize.root(equations, initial_guess, method='broyden2', tol=1e-9)
    
    print("\nSolution: ", solution.x)
    
    ans = input("Plot Ellipsoids? (y/n): ")
    if ans == "y":
       plot_hyperboloids_and_ellipsoid(a1, b1, c1, h1, k1, l1, a2, b2, c2, h2, k2, l2, a3, b3, c3, h3, k3, l3,[solution.x[0],solution.x[1],solution.x[2]])
    
    return solution.x

def calculate_ellipsoid_params(tx, rx_1, rx_2, rx_3, drone, error):
    """
    Calculate the ellipsoid parameters for three receivers. Note that for the c paramiter, im  not sure why it needs to be = b but we ball 
    """
    error_1 = rand.uniform(0, 1)
    error_2 = rand.uniform(0, 1)
    error_3 = rand.uniform(0, 1)
    
    l1 = m.sqrt((tx[0] - rx_1[0]) ** 2 + (tx[1] - rx_1[1]) ** 2 + (tx[2] - rx_1[2]) ** 2)
    l2 = m.sqrt((tx[0] - rx_2[0]) ** 2 + (tx[1] - rx_2[1]) ** 2 + (tx[2] - rx_2[2]) ** 2)
    l3 = m.sqrt((tx[0] - rx_3[0]) ** 2 + (tx[1] - rx_3[1]) ** 2 + (tx[2] - rx_3[2]) ** 2)
    
    print("\nDistance L1 =", l1)
    print("\nDistance L2 =", l2)
    print("\nDistance L3 =", l3)
    
    r_r1 = m.sqrt((drone[0] - rx_1[0]) ** 2 + (drone[1] - rx_1[1]) ** 2 + (drone[2] - rx_1[2]) ** 2)
    r_t = m.sqrt((drone[0] - tx[0]) ** 2 + (drone[1] - tx[1]) ** 2 + (drone[2] - tx[2]) ** 2)
    if error:
        r_r1 += error_1
        r_t += error_1
    
    print("\nDistance Rr1 =", r_r1)
    print("\nDistance Rt =", r_t)
    
    # Calculate parameters for the first ellipsoid
    a1 = (r_r1 + r_t) / 2
    b1 = m.sqrt(a1**2 - (l1**2 / 4))
    c1 = b1  # Modify this if you have a different c1
    
    h1 = (tx[0] + rx_1[0]) / 2
    k1 = (tx[1] + rx_1[1]) / 2
    l1 = (tx[2] + rx_1[2]) / 2
    
    r_r2 = m.sqrt((drone[0] - rx_2[0]) ** 2 + (drone[1] - rx_2[1]) ** 2 + (drone[2] - rx_2[2]) ** 2)
    if error:
        r_r2 += error_2
    print("\nDistance Rr2 =", r_r2)
    
    a2 = (r_r2 + r_t) / 2
    b2 = m.sqrt(a2**2 - (l2**2 / 4))
    c2 = b2  
    
    h2 = (tx[0] + rx_2[0]) / 2
    k2 = (tx[1] + rx_2[1]) / 2
    l2 = (tx[2] + rx_2[2]) / 2
    
    r_r3 = m.sqrt((drone[0] - rx_3[0]) ** 2 + (drone[1] - rx_3[1]) ** 2 + (drone[2] - rx_3[2]) ** 2)
    if error:
        r_r3 += error_3
    print("\nDistance Rr3 =", r_r3)
    
    a3 = (r_r3 + r_t) / 2
    b3 = m.sqrt(a3**2 - (l3**2 / 4))
    c3 = b3  # Modify this if you have a different c3
    
    h3 = (tx[0] + rx_3[0]) / 2
    k3 = (tx[1] + rx_3[1]) / 2
    l3 = (tx[2] + rx_3[2]) / 2
    
    print("\nParameters of Ellipsoid 1:", (a1, b1, c1, h1, k1, l1))
    print("\nParameters of Ellipsoid 2:", (a2, b2, c2, h2, k2, l2))
    print("\nParameters of Ellipsoid 3:", (a3, b3, c3, h3, k3, l3))
    
    return a1, b1, c1, h1, k1, l1, a2, b2, c2, h2, k2, l2, a3, b3, c3, h3, k3, l3

def calculate_hyperboloid_params(tx, rx_1, rx_2, rx_3, drone, error):
    """
    Calculate the hyperboloid parameters for three receivers and transmitter positions.
    """
    error_1 = rand.uniform(0, 1)
    error_2 = rand.uniform(0, 1)
    error_3 = rand.uniform(0, 1)

    l1 = m.sqrt((tx[0] - rx_1[0]) ** 2 + (tx[1] - rx_1[1]) ** 2 + (tx[2] - rx_1[2]) ** 2)
    l2 = m.sqrt((tx[0] - rx_2[0]) ** 2 + (tx[1] - rx_2[1]) ** 2 + (tx[2] - rx_2[2]) ** 2)
    l3 = m.sqrt((tx[0] - rx_3[0]) ** 2 + (tx[1] - rx_3[1]) ** 2 + (tx[2] - rx_3[2]) ** 2)

    print("\nDistance L1 =", l1)
    print("\nDistance L2 =", l2)
    print("\nDistance L3 =", l3)

    r_r1 = m.sqrt((drone[0] - rx_1[0]) ** 2 + (drone[1] - rx_1[1]) ** 2 + (drone[2] - rx_1[2]) ** 2)
    r_t = m.sqrt((drone[0] - tx[0]) ** 2 + (drone[1] - tx[1]) ** 2 + (drone[2] - tx[2]) ** 2)
    if error:
        r_r1 += error_1
        r_t += error_1

    print("\nDistance Rr1 =", r_r1)
    print("\nDistance Rt =", r_t)

    # Calculate parameters for the first hyperboloid
    a1 = (r_r1 - r_t) / 2  # Note the sign difference for hyperboloid
    b1 = m.sqrt(a1**2 + (l1**2 / 4))
    c1 = b1  # Modify this if you have a different c1

    h1 = (tx[0] + rx_1[0]) / 2
    k1 = (tx[1] + rx_1[1]) / 2
    l1 = (tx[2] + rx_1[2]) / 2

    r_r2 = m.sqrt((drone[0] - rx_2[0]) ** 2 + (drone[1] - rx_2[1]) ** 2 + (drone[2] - rx_2[2]) ** 2)
    if error:
        r_r2 += error_2
    print("\nDistance Rr2 =", r_r2)

    a2 = (r_r2 - r_t) / 2
    b2 = m.sqrt(a2**2 + (l2**2 / 4))
    c2 = b2

    h2 = (tx[0] + rx_2[0]) / 2
    k2 = (tx[1] + rx_2[1]) / 2
    l2 = (tx[2] + rx_2[2]) / 2

    r_r3 = m.sqrt((drone[0] - rx_3[0]) ** 2 + (drone[1] - rx_3[1]) ** 2 + (drone[2] - rx_3[2]) ** 2)
    if error:
        r_r3 += error_3
    print("\nDistance Rr3 =", r_r3)

    a3 = (r_r3 + r_t) / 2
    b3 = m.sqrt(a3**2 - (l3**2 / 4))
    c3 = b3  # Modify this if you have a different c3
    h3 = (tx[0] + rx_3[0]) / 2
    k3 = (tx[1] + rx_3[1]) / 2
    l3 = (tx[2] + rx_3[2]) / 2
    print("\nParameters of Ellipsoid 1:", (a1, b1, c1, h1, k1, l1))
    print("\nParameters of hyperboloid 1:", (a2, b2, c2, h2, k2, l2))
    print("\nParameters of Hyperboloid 2:", (a3, b3, c3, h3, k3, l3))

    return a1, b1, c1, h1, k1, l1,a2, b2, c2, h2, k2, l2,a3, b3, c3, h3, k3, l3
    
def plot_ellipsoids(h1, k1, l1, a1, b1, c1, h2, k2, l2, a2, b2, c2, h3, k3, l3, a3, b3, c3, solution,nu):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Define the grid for plotting the ellipsoids
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    
    # Plot Ellipsoid 1
    x1 = h1 + a1 * np.outer(np.cos(u), np.sin(v))
    y1 = k1 + b1 * np.outer(np.sin(u), np.sin(v))
    z1 = l1 + c1 * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x1, y1, z1, color='purple', alpha=0.3)
    
    # Plot Ellipsoid 2
    x2 = h2 + a2 * np.outer(np.cos(u), np.sin(v))
    y2 = k2 + b2 * np.outer(np.sin(u), np.sin(v))
    z2 = l2 + c2 * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x2, y2, z2, color='g', alpha=0.3)
    
    # Plot Ellipsoid 3
    x3 = h3 + a3 * np.outer(np.cos(u), np.sin(v))
    y3 = k3 + b3 * np.outer(np.sin(u), np.sin(v))
    z3 = l3 + c3 * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x3, y3, z3, color='b', alpha=0.3)
    
    # Plot the solution points
    if nu == True: 
        ax.scatter(solution[0], solution[1], solution[2], color='purple', s=100)

    else:
        for sol in solution:
            if isinstance(sol, tuple):
                x_sol, y_sol, z_sol = sol
                ax.scatter(x_sol.evalf(), y_sol.evalf(), z_sol.evalf(), color='r', s=100,alpha=1, edgecolors='black')
    
    ax.set_xlabel('X axis m')
    ax.set_ylabel('Y axis m')
    ax.set_zlabel('Z axis m')
    ax.set_title('Ellipsoids range sum intersection points')
    
    plt.show()

def ellipsoid_eq(point, params):
    x, y, z = point
    a, b, c, h, k, l = params
    return [
        ((x - h)**2 / (a)**2) + ((y - k)**2 / (b)**2) + ((z - l)**2 / (c)**2) - 1
    ]

def combined_eq(point, ellipsoids):
    equations = []
    for params in ellipsoids:
        equations.append(ellipsoid_eq(point, params))
    return equations

def find_intersection(ellipsoids):
    def objective_function(point):
        return np.sum(np.square(combined_eq(point, ellipsoids)))

    initial_guess = np.zeros(3)
    result = minimize(objective_function, initial_guess, method='Nelder-Mead')

    return result.x

def plot_hyperboloids_and_ellipsoid(a1, b1, c1, h1, k1, l1, a2, b2, c2, h2, k2, l2, a3, b3, c3, h3, k3, l3, solution=None):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot hyperboloid 1 (both tails)
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    
    # Positive tail
    x1_pos = a1 * np.outer(np.cos(u), np.sin(v)) + h1
    y1_pos = b1 * np.outer(np.sin(u), np.sin(v)) + k1
    z1_pos = c1 * np.outer(np.ones_like(u), np.cos(v)) + l1
    ax.plot_surface(x1_pos, y1_pos, z1_pos, color='b', alpha=0.3)

    # Negative tail
    x1_neg = -a1 * np.outer(np.cos(u), np.sin(v)) + h1
    y1_neg = -b1 * np.outer(np.sin(u), np.sin(v)) + k1
    z1_neg = -c1 * np.outer(np.ones_like(u), np.cos(v)) + l1
    ax.plot_surface(x1_neg, y1_neg, z1_neg, color='b', alpha=0.3)

    # Plot hyperboloid 2 (both tails)
    x2_pos = a2 * np.outer(np.cos(u), np.sin(v)) + h2
    y2_pos = b2 * np.outer(np.sin(u), np.sin(v)) + k2
    z2_pos = c2 * np.outer(np.ones_like(u), np.cos(v)) + l2
    ax.plot_surface(x2_pos, y2_pos, z2_pos, color='g', alpha=0.3)

    x2_neg = -a2 * np.outer(np.cos(u), np.sin(v)) + h2
    y2_neg = -b2 * np.outer(np.sin(u), np.sin(v)) + k2
    z2_neg = -c2 * np.outer(np.ones_like(u), np.cos(v)) + l2
    ax.plot_surface(x2_neg, y2_neg, z2_neg, color='g', alpha=0.3)

    # Plot ellipsoid
    x3 = a3 * np.outer(np.cos(u), np.sin(v)) + h3
    y3 = b3 * np.outer(np.sin(u), np.sin(v)) + k3
    z3 = c3 * np.outer(np.ones_like(u), np.cos(v)) + l3
    ax.plot_surface(x3, y3, z3, color='r', alpha=0.3)

    # Plot solution point if provided
    if solution:
        ax.scatter(solution[0], solution[1], solution[2], color='k', s=100, label='Drone')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('TDOA Curves vs  Range Sum')
    ax.legend()

    plt.show()



if __name__ == "__main__":
    #Example for quick demonstration

    calculate_position_mathematically_hyperboloid((4,3,3),(1,1,1),(2,2,2),(5,5,5),(6,6,6))
    
    calculate_position_numericaly_ellipsoid((1, 0, 0),(9,4, 1),(-3, 10, -11),(1, 9, 3),(6,7,9))

    calculate_position_mathematically_ellipsoid((0, 0, 0),(1, 0, 0),(0, 1, 0),(0, 0, 1),(0.5,0.5,0.5))

    calculate_position_numericaly_ellipsoid((0, 0, 0),(1, 0, 0),(0, 1, 0),(0, 0, 1),(0.5,0.5,0.5))