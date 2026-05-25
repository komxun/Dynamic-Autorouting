# IFDS Dynamic Autorouting

**Interfered Fluid Dynamical System (IFDS) for 3-D UAV Path Planning with Weather Constraints**

Results Video: https://youtu.be/XtmcNa-w4-0?si=V0FAj7HmrgcvlQuK

![image](https://github.com/komxun/IFDS-Algorithm/assets/133139057/078c3a5d-717b-4cf6-a459-22dee9d5c450)

## Quick Start

1. Open MATLAB and set this repository as the working directory.
2. Edit `default_config.m` to set scene, UAV parameters, weather gain, etc.
3. Run `main.m`.

The script automatically adds `src/`, `plots/`, and `data/` to the path.

## Repository Structure

```
IFDS-Algorithm/
├── main.m                  % Entry point (config-driven)
├── default_config.m        % All tunable parameters
├── src/                    % Core algorithm & helpers
│   ├── IFDS.m              % Main IFDS path planner
│   ├── create_scene.m      % Scene obstacle definitions
│   ├── create_shape.m      % Unified shape primitive
│   ├── apply_weather.m     % Weather constraint coupling
│   ├── calc_ubar.m         % Modulated velocity computation
│   ├── path_optimizing.m   % Global path optimizer (fmincon)
│   ├── setup_simulation.m  % Initialises all data structures
│   ├── initialize_constraint_matrix.m  % Weather data loader
│   ├── SE3Track.m          % SE(3) geometric path tracker
│   ├── hold_position.m     % SE(3) hover controller
│   ├── se3_controller_step.m  % Shared SE(3) controller core
│   ├── DirtyDerivative.m   % Numerical differentiator
│   ├── hat.m               % Skew-symmetric (hat) map
│   ├── vee.m               % Inverse hat map
│   └── norm_ubar.m         % Objective for local optimizer
├── plots/                  % Visualisation scripts
│   ├── PlotObject.m
│   ├── PlotPath.m
│   ├── PlotQuadcopter.m
│   ├── plotting_everything.m
│   └── se3_plot.m
├── data/                   % .mat data files
├── docs/                   % Technical report & proofs
├── tools/                  % Analysis & generation utilities
├── legacy/                 % Superseded scripts (kept for reference)
└── figures/                % Saved figure outputs
```

## Features

- **IFDS velocity-field planner** with super-ellipsoid obstacle avoidance
- **Weather constraint matrix** integration (dynamic or static)
- **SE(3) geometric tracking controller** (Lee et al. 2010/2011)
- **Global & Local path optimisation** via `fmincon`
- **Multi-target** and **dynamic-obstacle** scenarios

## Known Limitations

- Overlapped obstacle shapes can produce invalid paths
- Non-uniform barrier for cylinders/cones (safeguard derived from sphere)
- Stagnation near surfaces orthogonal to the path direction

## References

- Lee, T. et al. "Geometric tracking control of a quadrotor UAV on SE(3)" (2010), arXiv:1003.2005
- Komsun Tamanakijprasart, "IFDS Dynamic Autorouting" (2023)
