import numpy as np
import math

def covariance(P0, K, d_d, theta_k):
    Fx = np.array([[1,0,-d_d*math.sin(theta_k)],[0,1,d_d*math.cos(theta_k)],[0,0,1]])  #jacobian matrix
    Fv = np.array([[math.cos(theta_k), 0],[math.sin(theta_k), 0],[0,1]])
    a = 1.5
    V = a*np.array([[0.05**2, 0],[0, math.radians(0.5)**2]])  # odom noise variance, with standard deviation 5cm and 0.5 degree
    for k in range(K):
        Pk = np.dot(np.dot(Fx, P0), np.transpose(Fx)) + np.dot(np.dot(Fv, V), np.transpose(Fv))
        P0 = Pk

    return Pk



if __name__ == "__main__":
    P0 = np.array([[0.005**2,0,0],[0,0.005**2,0],[0,0,0.001**2]])  # initial covariance
    K = 700  # total time (seconds)
    d_d = 0.1  # velocity (m/s)
    theta_k = math.radians(45)  # Angular velocity (rad/s)
    Pk = covariance(P0, K, d_d, theta_k)
    print(Pk)
    uncertainty = math.sqrt(np.linalg.det(Pk))
    print(uncertainty)









