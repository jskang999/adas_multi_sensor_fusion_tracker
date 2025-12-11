# Design Notes: Multi Sensor Fusion Tracker

## State Model

- State vector: x = [x, y, vx, vy]^T
- Constant velocity model:

  x_k+1 = F x_k + w_k

  with

  F = [[1, 0, dt, 0],
       [0, 1, 0, dt],
       [0, 0, 1,  0],
       [0, 0, 0,  1]]

  Process noise Q is constructed from a simple 1D constant acceleration model
  applied independently to x and y.

## Sensors

### Camera

- Measurement: z_cam = [x, y]^T
- Linear measurement model:

  z_cam = H_cam x + v_k

  H_cam = [[1, 0, 0, 0],
           [0, 1, 0, 0]]

### Radar

- Measurement: z_rad = [r, phi, r_dot]^T

  r     = sqrt(x^2 + y^2)  
  phi   = atan2(y, x)  
  r_dot = (x * vx + y * vy) / r

- Nonlinear measurement → Extended Kalman Filter
- Jacobian H_jacobian(x) is used for the update step.

## Data Association

- For each track and detection pair, compute the squared Mahalanobis distance
  between the predicted measurement and the actual measurement.
- A greedy nearest-neighbor association is used on the cost matrix, with a
  Mahalanobis distance gate.
- Unassigned detections start new tracks, while tracks that remain unassigned
  increase their `missed` counter and are eventually removed.

## Track Management

- Tracks start as unconfirmed.
- A track becomes confirmed after `min_hits_to_confirm` steps.
- Tracks are removed if `missed > max_missed`.

## Simulation

- A simple 2D highway scenario generates multiple objects with constant velocity.
- A sensor simulator creates noisy camera and radar measurements with missed
  detections and clutter.
- A command-line app runs the simulation and writes:

  - `ground_truth.csv`
  - `tracks.csv`
  - `detections.csv`

  into an `output/` directory at the project root, which can be visualized with
  `tools/plot_tracks.py`.
