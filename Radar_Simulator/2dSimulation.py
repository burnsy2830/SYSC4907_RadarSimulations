import random as rand
import math as m
import numpy as np
from scipy.optimize import minimize
import sympy as sp
from matplotlib import pyplot as plt
from math import pi

def cli():
    #transmiter 
    #input_tx = input("Enter Location of Tx in the format x,y no brackets")
    #parts = input_tx.split(',')
    #tx = tuple(int(x) for x in parts)
    #reciver 1
    #input_rx1 = input("Enter Location of Rx_1 in the format x,y no brackets")
    #parts = input_rx1.split(',')
    #rx_1 = tuple(int(x) for x in parts)
    #reciver 2
    #input_rx2 = input("Enter Location of Rx_2 in the format x,y no brackets")
    #parts = input_rx2.split(',')
    #rx_2 = tuple(int(x) for x in parts)
    #Drone Location
    #print("\n Gennorating Random location of drone ..")
    #rand_1 = rand.randint(1,100)
    #rand_2 = rand.randint(1,100)
    #drone = (rand_1,rand_2)

    option = input("Enter Simulation Method h for TDOA , E for range sum, m for more options")
    if option == "H":
        calculate_postion_mathematically_hyperbola(tx,rx_1,rx_2,drone)
    if option == "E":
        select = input("N for numerically, M for mathematically")
        if select == "M":
            calculate_position_mathematically_ellipse(tx=(5,0),rx_1=(-5,0),rx_2=(-11,0), drone=(0,m.sqrt(75)))
        if select == "N":
            calculate_position_numerically_ellipse(tx=(5,0),rx_1=(-5,0),rx_2=(-11,0), drone=(0,m.sqrt(75)))
            
    else:
        raise NotImplementedError

    

def calculate_postion_mathematically_hyperbola(tx,rx_1,rx_2,drone):
    raise NotImplementedError
    

def calculate_position_numerically_ellipse(tx, rx_1, rx_2, drone,error=True):
    """
    Call the calcualte ellipse paramiters, determine the distance (also add some error) then use numerical methods to solve the closest point to both ellipses
    In the real world, youd imagibe that there will be some error on Rr or Rt calculation, depending on the size of this error this could cause there to be no real solution to the two ellipses,
    In this case we use numerical methods to get a good estimate of the closest point between the two ellipses. 
    
    """
    a1, b1, k1, h1, a2, b2, h2, k2 = calculate_ellipse_params(tx, rx_1, rx_2, drone,True)
    initial_guess = np.array([0.0, 0.0])
    bounds = ((0, None), (0, None))  # This specifies x >= 0 and y >= 0
    result = minimize(lambda point: distance_from_ellipses(point, a1, b1, h1, k1, a2, b2, h2, k2), initial_guess, bounds=bounds,method='trust-constr')
    x_opt, y_opt = result.x
    print("SOL:", x_opt,y_opt)
    ans = input("Plot Ellipses? (y/n)")
    if ans == "y":
        plot_ellipses(h1,k1,a1,b1,h2,k2,a2,b2)

    else:

        return x_opt, y_opt


def calculate_position_mathematically_ellipse(tx, rx_1, rx_2, drone):
    """
    Calculates the Ellipses Intercepts using mathamatical methods, not taking tino accound error (IE we are not using numerical methods), this will not take into account error, it assumes the range calucations 
    are 100% accurate. It just simply solves for the interception.

    """

    a1, b1, k1, h1, a2, b2, h2, k2 = calculate_ellipse_params(tx, rx_1, rx_2, drone,False) 
    x = sp.Symbol('x', real=True)
    y = sp.Symbol('y', real=True)
    ellipse1_eq = (((x-h1))**2 / a1**2) + ((y-k1)**2 / b1**2) - 1
    ellipse2_eq = (((x-h2))**2 / a2**2) + ((y-k2)**2 / b2**2) - 1
    solution = sp.solve((ellipse1_eq, ellipse2_eq), (x, y))
    print("\n Sol: ", solution)
    ans = input("Plot Ellipses? (y/n)")
    if ans == "y":
        plot_ellipses(h1,k1,a1,b1,h2,k2,a2,b2)

    else:

        return solution


def calculate_ellipse_params(tx, rx_1, rx_2, drone,error):
    """
    Just regular old calcuations of ellipse paramiters (semi major, semi minor, center ext). See my document for how some things are calculated
    """
    error_1 = rand.uniform(10, 20)
    error_2 = rand.uniform(0, 1)
    l1 = m.sqrt((tx[0] - rx_1[0])**2 + (tx[1] - rx_1[1])**2)
    l2 = m.sqrt((tx[0] - rx_2[0])**2 + (tx[1] - rx_2[1])**2)

    print("\n Distance L1 =", l1)
    print("\n Distance L2 =", l2)

    # Calculate distances from drone to Rx1 and Tx
    r_r1 = m.sqrt((drone[0] - rx_1[0])**2 + (drone[1] - rx_1[1])**2)
    r_t = m.sqrt((drone[0] - tx[0])**2 + (drone[1] - tx[1])**2)
    if error == True: 
        r_r1 = r_r1 + error_1
        r_t = r_r1 + error_1

    print("\n Distance Rr1 =", r_r1)
    print("\n Distance Rt =", r_t)

    # Calculate parameters for the first ellipse


    #Major + Minor Axis  EL1
    a1 = (r_r1 + r_t) / 2
    b1 = m.sqrt(a1**2 - (l1**2 / 4))

    
    h1 = (abs(abs(tx[0]) - abs(rx_1[0])))/2
    k1 = (abs(abs(tx[1]) - abs(rx_1[1])))/2


    # Calculate Ellipse 2 params 

    r_r2 = m.sqrt((drone[0] - rx_2[0])**2 + (drone[1] - rx_2[1])**2)
    if error == True: 
        r_r2 = r_r2 + error_2
    print("\n Distance Rr2 =", r_r2)

    a2 = (r_r2 + r_t) / 2
    b2 = m.sqrt(a2**2 - (l2**2 / 4))


    h2 = (abs(abs(tx[0]) - abs(rx_2[0])))/2
    k2 = (abs(abs(tx[1]) - abs(rx_2[1])))/2



    print("\n Parameters of Ellipse 1:", (a1, b1, h1, k1))
    print("\n Parameters of Ellipse 2:", (a2, b2, h2, k2))
    
    return a1, b1, k1, h1, a2, b2, h2, k2


def distance_from_ellipses(point, a1, b1, h1, k1, a2, b2, h2, k2):
    x, y = point
    dist1 = (((x - h1) / a1) ** 2 + ((y - k1) / b1) ** 2 - 1)**2
    dist2 = (((x - h2) / a2) ** 2 + ((y - k2) / b2) ** 2 - 1)**2
    return dist1 + dist2


def plot_ellipses(u1, v1, a1, b1, u2, v2, a2, b2):
    """
    This just provides a visual for the ellipses, will show the overlap. It allows you to verify answers genorated by both methods.
    """
    t = np.linspace(0, 2*pi, 100)
    plt.plot(u1 - a1 * np.cos(t), v1 + b1 * np.sin(t), label='Ellipse 1')
    plt.plot(u2 - a2 * np.cos(t), v2 + b2 * np.sin(t), label='Ellipse 2')
    plt.grid(color='lightgray', linestyle='--')
    plt.legend()
    plt.axis('equal')
    plt.xlabel('X-axis (m)')
    plt.ylabel('Y-axis (m)')
    plt.title('Bistatic Elipse Overlap')
    plt.show()

if __name__ == "__main__":
    cli()

   


    







    
