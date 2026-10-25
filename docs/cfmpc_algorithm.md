# cf-MPC Algorithm Documentation

## Reference

Hou, Fernandez, Zhu, Hong — *"Model Predictive Control with Visibility Graphs
for Humanoid Path Planning and Tracking Against Adversarial Opponents"*
ICRA 2025, arXiv 2504.02184, Section 3.

## QP Formulation

At each time step, the cf-MPC controller solves the following Quadratic
Program (QP) over a prediction horizon of N steps:

### Objective

```
min  Σ_k [ ||u_k||²_R  +  ρ · Σ_j σ²_{k,j} ]  +  ||p_N − p_ref_N||²_{Q_N}
```

### Constraints

| Constraint | Description |
|---|---|
| `x_{k+1} = A x_k + B u_k` | Discrete linear dynamics (omnidirectional) |
| `\|u_k\| ≤ u_max` | Actuator bounds (per element) |
| `(p_k − c_j) · n_j ≥ d_j − σ_{k,j}` | Tangent-plane obstacle avoidance |
| `σ_{k,j} ≥ 0` | Non-negative slack |

### Variables

| Symbol | Meaning |
|---|---|
| `x = [p_x, p_y, θ]` | State (position + heading) |
| `u = [v_x, v_y, ω]` | Control (body-frame velocity) |
| `n_j, d_j` | Normal and offset of tangent plane for obstacle j |
| `ρ = cfg.rho_slack` | Quadratic slack penalty weight |
| `σ_{k,j}` | Slack variable for obstacle j at step k |

## QP Cost Weights

| Weight | Value | Role |
|---|---|---|
| `W_POS` | 10.0 | Tracking cost per horizon step |
| `W_VEL` | 1.0 | Control regularization |
| `W_TERM` | 50.0 | Terminal cost (final position → reference) |

## Key Design Decisions

### Slack Variables (PRD §4.2)
Slack penalty is **quadratic** (`ρ · Σ σ²`) to ensure the QP remains feasible
even when the robot is inside an obstacle's safe zone. This guarantees a
solution is always found, with the penalty discouraging constraint violation.

### Parameter Caching
CVXPY `Parameter` objects are used to compile the QP once per obstacle count,
then reuse it across time steps. This reduces solve time to ≈2.5 ms,
satisfying the PRD K1 requirement (60 FPS).

### Velocity Clipping
Output velocity is clipped by **Euclidean norm** (not per-component) to
respect the robot's physical speed limit consistently.

### Solver Fallback
If the QP solver fails unexpectedly, the controller falls back to
`FollowerController` to maintain robustness.

## Configuration Parameters

| Parameter | Source | Usage |
|---|---|---|
| `horizon` | `cfg` | Prediction horizon length N |
| `dt_mpc` | `cfg` | Time step size (seconds) |
| `max_speed` | `cfg` | Translational speed limit (m/s) |
| `max_omega` | `cfg` | Rotational speed limit (rad/s) |
| `rho_slack` | `cfg` | Slack penalty weight in cost function |
| `robot_radius + safety_margin` | `cfg` | Builds `d_j` for tangent planes |

## Dependencies

- `cvxpy >= 1.4` (optional; `control/__init__.py` sets `CfMpcController = None` if missing)
- `numpy`

