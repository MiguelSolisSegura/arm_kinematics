#!/usr/bin/env python3
from math import atan2, acos, sqrt, pi, cos, sin

# Joint limits in radians
theta2_min = -pi / 4
theta2_max = 3 * pi / 4

theta3_min = -3 * pi / 4
theta3_max = 3 * pi / 4

# Link lengths
r2 = 1.0
r3 = 1.0

# Given position of Frame 3 origin
P3 = [0.5, 0.6, 0.7]
Px, Py, Pz = P3

# Inverse kinematics calculations
# Calculate two possible theta1 values
theta1_candidate1 = atan2(Py, Px)
theta1_candidate2 = theta1_candidate1 + pi

# Calculate r1, which is the projection of the link 2 and link 3 combination on the x-y plane
r1 = sqrt(Px**2 + Py**2)

# Calculate d, which is the vertical distance between the base and the Frame 3 origin
d = Pz

# Calculate the distance between the base of the second joint and the origin of Frame 3
D = sqrt(r1**2 + d**2)

# Use cosine rule to calculate two possible theta2 values
cos_theta2 = (r2**2 + D**2 - r3**2) / (2 * r2 * D)

if cos_theta2 > 1 or cos_theta2 < -1:
    raise ValueError("Unreachable position. The desired position cannot be attained.")

theta2_candidate1 = atan2(d, r1) + acos(cos_theta2)
theta2_candidate2 = atan2(d, r1) - acos(cos_theta2)

# Calculate two possible theta3 values
cos_theta3 = (r2**2 + r3**2 - D**2) / (2 * r2 * r3)

if cos_theta3 > 1 or cos_theta3 < -1:
    raise ValueError("Unreachable position. The desired position cannot be attained.")

theta3_candidate1 = acos(cos_theta3) - pi
theta3_candidate2 = -acos(cos_theta3) - pi

# Normalize angles between -pi and pi
def normalize_angle(angle):
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

# Normalize all solutions
theta1_candidate1 = normalize_angle(theta1_candidate1)
theta1_candidate2 = normalize_angle(theta1_candidate2)
theta2_candidate1 = normalize_angle(theta2_candidate1)
theta2_candidate2 = normalize_angle(theta2_candidate2)
theta3_candidate1 = normalize_angle(theta3_candidate1)
theta3_candidate2 = normalize_angle(theta3_candidate2)

# Check feasibility of each candidate
def check_feasibility(theta2, theta3):
    feasible = True
    if not (theta2_min <= theta2 <= theta2_max):
        feasible = False
    if not (theta3_min <= theta3 <= theta3_max):
        feasible = False
    return feasible

# Recompute and verify X and Y positions
def validate_position(theta1, theta2, theta3):
    x = r2 * cos(theta1) * cos(theta2) + r3 * cos(theta1) * cos(theta2 + theta3)
    y = r2 * sin(theta1) * cos(theta2) + r3 * sin(theta1) * cos(theta2 + theta3)
    return (abs(x - Px) < 1e-6) and (abs(y - Py) < 1e-6)

# Define candidate solutions
solutions = [
    ("Solution 1", theta1_candidate1, theta2_candidate1, theta3_candidate1),
    ("Solution 2", theta1_candidate1, theta2_candidate2, theta3_candidate2),
    ("Solution 3", theta1_candidate2, theta2_candidate1, theta3_candidate1),
    ("Solution 4", theta1_candidate2, theta2_candidate2, theta3_candidate2),
]

# Display results and check feasibility
for name, theta1, theta2, theta3 in solutions:
    feasible = check_feasibility(theta2, theta3)
    valid_position = validate_position(theta1, theta2, theta3)
    feasibility_text = "feasible" if feasible else "not feasible"
    position_text = "valid" if valid_position else "invalid"
    print(f"{name}:")
    print(f"  Theta1: {theta1} radians")
    print(f"  Theta2: {theta2} radians")
    print(f"  Theta3: {theta3} radians")
    print(f"  This solution is {feasibility_text} and the position is {position_text}.\n")
