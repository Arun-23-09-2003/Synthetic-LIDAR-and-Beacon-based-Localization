# NavState Lie Group

The NavState helper functions are in `src/utils/`.

## Main Functions

- `make_navstate.m`: builds a `4 x 4` NavState matrix from heading, position, and velocity.
- `navstate_components.m`: extracts heading, position, and velocity from a NavState matrix.
- `navstate_exp.m`: implements the required exponential map contract.
- `rotation2d.m`: returns a 2D rotation matrix.
- `wrap_pi.m`: wraps angles to `[-pi, pi)`.

## Exponential Map Contract

`navstate_exp(delta)` accepts:

```matlab
delta = [dtheta; dpx; dpy; dvx; dvy];
```

It uses the third-order Taylor left-Jacobian approximation from the README:

```matlab
A = 1 - phi^2 / 6;
B = phi / 2 - phi^3 / 24;
J = [A, -B; B, A];
```

The returned `dX` is a valid `4 x 4` NavState group element.

## Filter Usage

The EKF covariance lives in tangent coordinates ordered as:

```matlab
[delta_theta; delta_px; delta_py; delta_vx; delta_vy]
```

Measurement updates compute a world-frame tangent correction and then map position and velocity corrections into local group coordinates before applying:

```matlab
X_plus = X * navstate_exp(delta_group)
```

## Important Assumptions

The nominal state is always stored as a NavState matrix. Covariance and Kalman gains are computed in the local tangent-space error state.
