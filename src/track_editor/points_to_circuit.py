from scipy.interpolate import splprep, splev
import numpy as np
import json
import math


def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def translation(cone_list):
    origin = cone_list[0]
    traslated_cones = []

    for cone in cone_list:
        translation_x = cone[0] - origin[0]
        translation_y = cone[1] - origin[1]
        traslated_cones.append((translation_x, translation_y))

    return traslated_cones

def rotation(cone_list):
    # Get the first two cones
    x1, y1 = cone_list[0]
    x2, y2 = cone_list[1]

    # Calculate the difference in x and y
    dx = x2 - x1
    dy = y2 - y1

    # Calculate the rotation angle to align the first two cones with the X-axis
    rotation_angle = np.arctan2(dy, dx)

    # Rotation function: rotates a point (x, y) around the origin by the specified angle
    def rotate_point(x, y, angle):
        x_rotated = x * np.cos(angle) - y * np.sin(angle)
        y_rotated = x * np.sin(angle) + y * np.cos(angle)
        return x_rotated, y_rotated

    # Apply the rotation to all cones
    rotated_cones = [rotate_point(x, y, -rotation_angle) for x, y in cone_list]

    return rotated_cones

def smooth_and_expand_points(points, offset, num_points, min_distance=5):
    translated_points = translation(points)
    rotated_points = rotation(translated_points)

    x = []
    y = []

    for point in rotated_points:
        x.append(point[0])
        y.append(point[1])
    
    # Smoothed curve
    tck, u = splprep([x, y], s=1.0, per=1, k=3)
    
    u_new = np.linspace(0, 1, num_points)
    x_smooth, y_smooth = splev(u_new, tck)


    trajectory_json_data = {
        "x": [x for x in x_smooth],
        "y": [y for y in y_smooth],
        "s": [],
        "k": [],
        "speed_profile": [],
        "acc_profile": []
    }


    # Calculate the tangents (derivatives) for each point of the smooth line
    dx, dy = splev(u_new, tck, der=1)
    
    # Normalize the tangents and calculate the normal vectors
    norm = np.sqrt(dx**2 + dy**2)
    normal_x, normal_y = -dy / norm, dx / norm

    outer_cones = []
    inner_cones = []

    # Add the first outer and inner points
    for i in range(num_points):
        outer_point = (x_smooth[i] + offset/2 * normal_x[i], 
                       y_smooth[i] + offset/2 * normal_y[i])
        inner_point = (x_smooth[i] - offset/2 * normal_x[i], 
                       y_smooth[i] - offset/2 * normal_y[i])
        
        # If it is the first cone or the distance between the last point in the list and the one to be added is greater than or equal to the minimum distance, add the point
        if i == 0 or distance(outer_cones[-1], outer_point) >= min_distance:
            outer_cones.append(outer_point)
        if i == 0 or distance(inner_cones[-1], inner_point) >= min_distance:
            inner_cones.append(inner_point)

    # Calculate json data for the circuit
    acum = 0.0
    s = [0.0]
    xp, yp = [], []
    for i in range(len(x_smooth) - 1):
        dx = x_smooth[i+1] - x_smooth[i]
        dy = y_smooth[i+1] - y_smooth[i]
        dist = math.sqrt(dx*dx + dy*dy)
        xp.append(dx)
        yp.append(dy)
        acum += dist
        s.append(acum)
    xp.append(xp[-1])
    yp.append(yp[-1])

    xpp, ypp = [], []
    for i in range(len(xp) - 1):
        xpp.append(xp[i+1] - xp[i])
        ypp.append(yp[i+1] - yp[i])
    xpp.append(xpp[-1])
    ypp.append(ypp[-1])

    k = []
    for i in range(len(xpp)):
        den = ((xp[i]**2 + yp[i]**2)**1.5)
        val = 0 if den == 0 else (xp[i]*ypp[i] - xpp[i]*yp[i]) / den
        k.append(val)

    # Speed limits
    AY_MAX, AX_MAX, V_MAX = 4.0, 5.0, 15.0
    speed_profile = [0.0 for _ in s]
    v_grip = [min(math.sqrt(AY_MAX / max(abs(val), 1e-4)), V_MAX) for val in k]
    speed_profile[0] = 1.0

    for j in range(1, len(speed_profile)):
        ds = s[j] - s[j-1]
        speed_profile[j] = math.sqrt(speed_profile[j-1]**2 + 2*AX_MAX*ds)
        if speed_profile[j] > v_grip[j]:
            speed_profile[j] = v_grip[j]
    for j in range(len(speed_profile)-2, -1, -1):
        ds = s[j+1] - s[j]
        v_brake = math.sqrt(speed_profile[j+1]**2 + 2*AX_MAX*ds)
        if speed_profile[j] > v_brake:
            speed_profile[j] = v_brake

    acc_profile = [0.0]*len(speed_profile)
    for i in range(1, len(speed_profile)):
        ds = s[i] - s[i-1]
        if ds != 0:
            acc_profile[i] = (speed_profile[i] - speed_profile[i-1]) / (ds/speed_profile[i])

    trajectory_json_data = {
        "x": list(x_smooth),
        "y": list(y_smooth),
        "s": s,
        "k": k,
        "speed_profile": speed_profile,
        "acc_profile": acc_profile
    }

    return outer_cones, inner_cones, trajectory_json_data