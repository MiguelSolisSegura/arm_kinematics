#!/usr/bin/env python3
from math import atan2, acos, sqrt, pi, cos, sin

# Link lengths
r2 = 1.0
r3 = 1.0

# Given position of Frame 3 origin
P3 = [0.5, 0.6, 0.7]
Px, Py, Pz = P3

# Inverse kinematics calculations
# Calculate two possible theta1 values
theta1_candidate1 = atan2(Py, Px)
theta1_candidate2 = theta1_candidate1 - pi

def normalize_angle(angle):
    # Normalize the angle to be within the range -pi to pi
    normalized_angle = (angle + pi) % (2 * pi) - pi
    return normalized_angle

# Function to compute `theta2` and `theta3` given `theta1`
def compute_theta2_and_theta3(theta1):
    # Compute the coordinates of the target relative to Frame 1
    P1x = Px * cos(theta1) + Py * sin(theta1)
    P1y = Pz

    # Compute the distance from the base of joint 2 to the target
    L = sqrt(P1x**2 + P1y**2)

    # Apply the cosine rule to find theta2
    cos_theta2 = (L**2 + r2**2 - r3**2) / (2 * r2 * L)
    theta2 = acos(cos_theta2)

    # Adjust theta2 for the correct quadrant
    theta2 = normalize_angle(atan2(P1y, P1x) - theta2)

    # Apply the cosine rule to find theta3
    cos_theta3 = (r2**2 + r3**2 - L**2) / (2 * r2 * r3)
    theta3 = acos(cos_theta3)

    # Adjust theta3 to align with the convention
    theta3 = normalize_angle(pi - theta3)

    return theta2, theta3

def compute_alternative_theta2_and_theta3(theta1):
    # Compute the coordinates of the target relative to Frame 1
    P1x = Px * cos(theta1) + Py * sin(theta1)
    P1y = Pz

    # Compute the distance from the base of joint 2 to the target
    L = sqrt(P1x**2 + P1y**2)

    # Apply the cosine rule to find theta2 (alternative)
    cos_theta2 = (L**2 + r2**2 - r3**2) / (2 * r2 * L)
    theta2 = acos(cos_theta2)

    # Adjust theta2 to flip the elbow up vs. elbow down
    theta2_alternative = normalize_angle(atan2(P1y, P1x) + theta2)

    # Apply the cosine rule to find theta3 (alternative)
    cos_theta3 = (r2**2 + r3**2 - L**2) / (2 * r2 * r3)
    theta3 = acos(cos_theta3)

    # Adjust theta3 to align with the convention
    theta3_alternative = normalize_angle(theta3 - pi)

    return theta2_alternative, theta3_alternative

def is_solution_possible(theta2, theta3):
    # Define the angle constraints
    lower_theta2 = -pi / 4
    upper_theta2 = 3 * pi / 4
    lower_theta3 = -3 * pi / 4
    upper_theta3 = 3 * pi / 4

    # Check if both theta2 and theta3 are within the specified range
    return lower_theta2 <= theta2 <= upper_theta2 and lower_theta3 <= theta3 <= upper_theta3


# Calculate the sets of angles for both theta1 candidates
theta1_set = [theta1_candidate1, theta1_candidate2]
solutions = []

for theta1 in theta1_set:
    theta2, theta3 = compute_theta2_and_theta3(theta1)
    solutions.append((theta1, theta2, theta3))
    theta2, theta3 = compute_alternative_theta2_and_theta3(theta1)
    solutions.append((theta1, theta2, theta3))

# Print both solutions in radians
for i, (theta1, theta2, theta3) in enumerate(solutions, start=1):
    print(f"Solution {i}:")
    print(f"   Theta1: {theta1} radians")
    print(f"   Theta2: {theta2} radians")
    print(f"   Theta3: {theta3} radians")
    print(f"   Solution possible: {is_solution_possible(theta2, theta3)}\n")
