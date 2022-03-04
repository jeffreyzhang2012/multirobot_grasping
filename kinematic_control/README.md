# Kinematic multi-robot control system

This work is based on the paper [Kinematic Multi-Robot Manipulation with no Communication Using Force Feedback](https://msl.stanford.edu/papers/wang_kinematic_2016.pdf).

## What this directory looks like
```
kinematic_control/
├── Project.prj
├── README.md
├── controller_follower.m
├── controller_leader.m
├── global_config.m
├── initial_spawn.m
├── main.m
├── object_dynamics.m
├── plot1d_helper.m
├── plot2d_helper.m
├── plot_force_helper.m
└── resources
```

## Configuration files
```global_config.m``` includes controller specifications and other important parameters.
```initial_spawn.m``` sets the initial state of the object.

## Run simulation
Run ```main.m``` to view the results.