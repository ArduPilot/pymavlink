

"""
This program is a Bayesian optimization algorithm that attempts to minimize the maximum 
angular velocity of a hovering copter. The algorithm uses Gaussian process regression to model 
the relationship between the drone's flight control parameters and the maximum angular velocity. 
It then uses an acquisition function to determine the next point to sample in order to improve 
the model's accuracy. The algorithm iteratively samples and updates the model until a satisfactory 
solution is found. This program has the potential to significantly improve the stability and 
control of copters by optimizing their flight control parameters.



@author:
   Haoqi Zeng
   Shanghai Jiao Tong University
   zeng-hq@sjtu.edu.cn

   Nanyang Ye
   Shanghai Jiao Tong University
   ynylincoln@sjtu.edu.cn

"""

import math
import time
import numpy as np
from numpy import argmax
from scipy.stats import norm
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Matern
from warnings import catch_warnings
from warnings import simplefilter
from pymavlink import mavutil
from collections import deque

###########################################################################################
"""
Note: 
Set the PID parameters' (minimum value, maximum value) properly 
before running the program to prevent any potential accidents, such as a crash.
"""

PID_bounds = {'PIT_P': (0, 0.5), 'PIT_I': (0, 0.5), 'PIT_D': (0, 0.5),
              'RLL_P': (0, 0.5), 'RLL_I': (0, 0.5), 'RLL_D': (0, 0.5),
              'YAW_P': (0, 0.5), 'YAW_I': (0, 0.5), 'YAW_D': (0, 0.5)}

###########################################################################################



# surrogate or approximation for the objective function
# This function is used to create a surrogate model that approximates the true objective function.
# It takes in a trained machine learning model and the input features, X, and returns a prediction and its standard deviation.
def surrogate(model, X):
    # catch any warning generated when making a prediction
    with catch_warnings():
        # ignore generated warnings
        simplefilter("ignore")
        return model.predict(X, return_std=True)
    
# probability of improvement acquisition function
# This function calculates the probability of improvement acquisition function for a given set of input features, X, a set of previously sampled features, X_samples, and a trained machine learning model, model.
def acquisition(X, X_samples, model):
    # calculate the best surrogate score found so far
    # Using the surrogate function, y_hat is calculated for the given set of input features X.
    # The maximum value of y_hat is considered as the best surrogate score found so far.
    y_hat, _ = surrogate(model, X)
    best = max(y_hat)
    # calculate mean and stdev via surrogate function
    # Using the surrogate function, the mean and standard deviation of the surrogate model are calculated for the previously sampled features, X_samples.
    mu, std = surrogate(model, X_samples)
    # calculate the probability of improvement
    # The probability of improvement is calculated by taking the cumulative distribution function of the normal distribution.
    # The argument of the cumulative distribution function is the difference between the mean of the surrogate model and the best surrogate score found so far, divided by the sum of the standard deviation of the surrogate model and a very small value to avoid division by zero.
    probs = norm.cdf((mu - best) / (std + 1E-9))
    return probs


# optimize the acquisition function
# This function optimizes the acquisition function to find the next set of input features to sample.
# It takes in the current set of input features, X, the corresponding output values, y, a trained machine learning model, model, and the bounds of the input features, bounds.
def opt_acquisition(X, y, model, bounds):
    # generate random samples
    # Using the random_9d_values function, 100 random samples are generated within the bounds of the input features.
    X_samples = random_9d_values(bounds, 100)
    # calculate the acquisition function for each sample
    # For each of the 100 random samples, the acquisition function is calculated using the acquisition function.
    scores = acquisition(X, X_samples, model)
    # locate the index of the largest scores
    # The index of the largest score is then found using the argmax function.
    ix = argmax(scores)
    # The input features corresponding to the index with the largest score are then returned.
    return X_samples[ix, :]



PID_params = ['ATC_RAT_PIT_P', 'ATC_RAT_PIT_I', 'ATC_RAT_PIT_D',
	      'ATC_RAT_RLL_P', 'ATC_RAT_RLL_I', 'ATC_RAT_RLL_D', 
	      'ATC_RAT_YAW_P', 'ATC_RAT_YAW_I', 'ATC_RAT_YAW_D']


def set_PID_param(param, value, connection):
    while True:
        # Set parameter
        connection.mav.param_set_send(
                connection.target_system, connection.target_component,
                param.encode(),
                value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                )
        
        # Request parameter
        connection.mav.param_request_read_send(
                connection.target_system, connection.target_component,
                param.encode(),
                -1
                )
        message = connection.recv_match(type='PARAM_VALUE', blocking=True).to_dict()

        # Break when success
        if math.isclose(value, message['param_value'], 
                        rel_tol=1e-4, abs_tol=1e-4):
            break

# This function generates n random 9-dimensional values within the specified bounds.
# It takes in a dictionary of PID parameter bounds and an optional number of values to generate.
def random_9d_values(PID_bounds, n=50):
    bounds = np.array([PID_bounds[key] for key in PID_bounds.keys()])
    values = []
    while len(values) < n:
        sample = np.random.rand(len(bounds))
        scaled_sample = (bounds[:,1] - bounds[:,0]) * sample + bounds[:,0]
        if all([low <= val <= high for (low, high), val in zip(bounds, scaled_sample)]):
            values.append(scaled_sample)
    return np.vstack(values)

# This function calculates the maximum pitch, roll, and yaw velocities of a connected MAVLink device.
# It takes in a connection object and an optional time duration in milliseconds.
def max_angular_velocities(connection, time_ms=5000):
    # Create three deques for pitch, roll, and yaw velocities
    pitch_velocities = deque(maxlen=int(time_ms/10))
    roll_velocities = deque(maxlen=int(time_ms/10))
    yaw_velocities = deque(maxlen=int(time_ms/10))

    # Extract pitch, roll, and yaw velocities from attitude message
    msg = connection.recv_match(type='ATTITUDE', blocking=True).to_dict()
    start_time = msg['time_boot_ms']
    pitch_velocities.append(abs(msg['pitchspeed']))
    roll_velocities.append(abs(msg['rollspeed']))
    yaw_velocities.append(abs(msg['yawspeed']))
    
    while msg['time_boot_ms'] - start_time < time_ms:
        msg = connection.recv_match(type='ATTITUDE', blocking=True).to_dict()
        pitch_velocities.append(abs(msg['pitchspeed']))
        roll_velocities.append(abs(msg['rollspeed']))
        yaw_velocities.append(abs(msg['yawspeed']))
    
    # Calculate the maximum velocity in each direction
    max_pitch_vel = max(pitch_velocities)
    max_roll_vel = max(roll_velocities)
    max_yaw_vel = max(yaw_velocities)

    return max_pitch_vel, max_roll_vel, max_yaw_vel


#####################    The function to be optimized. Can be customized.   ##########################
# 
"""
When given PID parameter values as input, this function optimizes the PID parameters by 
updating flight control parameters and calculating the maximum pitch velocity, roll velocity, 
and yaw velocity to achieve maximum flight control effectiveness. If the given PID parameter 
values have already been computed and stored in known results, the function returns the known 
result directly. If the given PID parameter values have not been computed before, the function 
updates the flight control parameters, calculates the maximum velocities, and stores the result 
in known results before returning the result. Finally, the function returns an array of results 
corresponding to the input PID parameter values.
"""
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
known_res = {}
def min_max_ang(PID_values):
    res = []
    PID_values_list = PID_values.tolist()
    if len(PID_values.shape) == 1:
        PID_values_list = [PID_values_list]
    for values_list in PID_values_list:
        values_tuple = tuple(values_list)

        if values_tuple in known_res:
            result = known_res(values_tuple)

        # Update flight control parameters and obtain result
        else:

            for param, value in zip(PID_params, values_list):
                set_PID_param(param, value, master)
            max_pitch_vel, max_roll_vel, max_yaw_vel = max_angular_velocities(master)
            result = max_pitch_vel + max_roll_vel + max_yaw_vel
            # result = np.random.rand()
            known_res[values_tuple] = result
       
        res.append(result)
        
    return np.array(res)


#######################################################################################################
def main():
    
    training_times = 50

    # Gaussian Process Regression Model
    kernel = Matern(length_scale=1, nu=2.5)
    model = GaussianProcessRegressor(kernel=kernel, alpha=0.01)

    # sample
    X = random_9d_values(PID_bounds)
    y = min_max_ang(X)
    # fit the model
    model.fit(X, y)

    # perform the optimization process
    for i in range(training_times):
        # select the next point to sample
        x = opt_acquisition(X, y, model, PID_bounds)
        # sample the point
        actual = min_max_ang(x)
        # summarize the finding
        est, _ = surrogate(model, x.reshape(1, -1))
        print('>x={}\n f()={}\n actual={}\n'.format(x, est, actual))
        # add the data to the dataset
        X = np.vstack((X, x.reshape(1, -1)))
        y = np.concatenate((y, actual))
        # update the model
        model.fit(X, y)

    # best result
    ix = np.argmin(y)
    print('Best Result: x={}, y={}'.format(X[ix], y[ix]))

if __name__ == '__main__':
    main()