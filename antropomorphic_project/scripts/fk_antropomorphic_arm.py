#!/usr/bin/env python3
from sympy import Matrix, cos, sin, Symbol, pi, simplify, trigsimp
from sympy.interactive import printing
from sympy import preview

# Initialize pretty printing
printing.init_printing(use_latex=True)

# Denavit-Hartenberg symbolic parameters
theta1 = Symbol("theta1")
theta2 = Symbol("theta2")
theta3 = Symbol("theta3")

alpha1 = pi / 2
alpha2 = 0
alpha3 = 0

r1 = 0
r2 = Symbol("r2")
r3 = Symbol("r3")

d1 = 0
d2 = 0
d3 = 0

# General DH transformation matrix template
def get_dh_matrix(theta, alpha, r, d):
    return Matrix([
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), r * cos(theta)],
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), r * sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Create individual transformation matrices
A01 = get_dh_matrix(theta1, alpha1, r1, d1)
A12 = get_dh_matrix(theta2, alpha2, r2, d2)
A23 = get_dh_matrix(theta3, alpha3, r3, d3)

# Chain the transformations to get the transformation matrix from base to Frame 3
A03 = A01 * A12 * A23
A03_simplify = trigsimp(simplify(A03))

# Ask the user to input the joint angles
theta1_value = float(input("Enter theta1 (in radians): "))
theta2_value = float(input("Enter theta2 (in radians): "))
theta3_value = float(input("Enter theta3 (in radians): "))

# Define fixed link lengths (modify as needed)
r2_value = 1.0
r3_value = 1.0

# Substitute in the numeric values
A03_evaluated = A03_simplify.subs({theta1: theta1_value, theta2: theta2_value, theta3: theta3_value, r2: r2_value, r3: r3_value})

# Extract position and orientation matrices
position_vector = A03_evaluated[:3, 3]
orientation_matrix = A03_evaluated[:3, :3]

# Display the evaluated position and orientation matrices
print("\nPosition Matrix:")
print(position_vector)
print("\nOrientation Matrix:")
print(orientation_matrix)

# Save the evaluated matrix as an image
preview(A03_evaluated, viewer='file', filename="A03_simplify_evaluated.png", dvioptions=['-D', '300'])
