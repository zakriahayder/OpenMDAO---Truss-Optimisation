# Truss Optimization Using OpenMDAO

## Overview

This repository presents a truss optimization problem implemented using [OpenMDAO](https://openmdao.org/), a framework for multidisciplinary optimization in Python. The objective is to design a truss structure that minimizes cost while satisfying constraints related to stress, buckling, and deflection.


## Problem Statement

Design a truss structure with the following objectives and constraints:

- **Objective:** Minimize the total cost of the truss.
- **Constraints:**
  - **Stress Constraint:** The stress in the truss members must not exceed the maximum allowable stress.
  - **Buckling Constraint:** The actual stress must not exceed the buckling stress of the truss members.
  - **Deflection Constraint:** The deflection at the top of the truss must not exceed the maximum allowable deflection.

## Mathematical Formulation

### Variables and Constants

- **Design Variables:**
  - \( H \): Height of the truss [meters].
  - \( d \): Diameter of the truss members [meters].
  
- **Constants:**
  - \( B  \): Base width of the truss [meters].
  - \( t \): Thickness of the truss members [meters].
  - \( E  \): Modulus of elasticity [MPa].
  - \( P \): Applied load [kN].
  - \( rho  \): Material density [kg/m³].
  - \( stress\_max \): Maximum allowable stress [MPa].
  - \( deflection\_max \): Maximum allowable deflection [meters].

### Equations

1. **Length of Truss Member ( L ):**

      L = sqrt((B / 2)^2 + H^2)

2. **Cross-sectional Area ( A ):**
   
      A = π * d * t

3. **Moment of Inertia over Area ( I / A ):**

      I / A = (d^2 + t^2) / 8

4. **Stress ( σ ):**
   
      σ = (P * L) / (2 * A * H)

5. **Deflection ( \delta ):**

      δ = (P * L^3) / (2 * E * A * H^2)

6. **Buckling Stress (\( \sigma_b \)):**
   
      σ_b = (π^2 * E * (I / A)) / L^2

7. **Cost (\( C \)):**

      C = 2 * ρ * A * L

### Constraints

1. **Stress Constraint:**

      σ ≤ stress_max

2. **Buckling Constraint:**

      σ ≤ σ_b

3. **Deflection Constraint:**

      δ ≤ deflection_max















