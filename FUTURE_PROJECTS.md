# Future Project Ideas

Ideas for repurposing the Intel NUC (N5105, 32GB RAM) - not used in Big RoverBot v1 (uses Pi 5 instead).

## Crawler Fleet Controller

Base station for multiple Pi-based crawlers:

```
                    Intel NUC (Base Station)
                           |
        +------------------+------------------+
        |                  |                  |
   Pi Crawler 1       Pi Crawler 2       Pi Crawler 3
```

Features:
- Centralized mission planning
- Aggregate video processing
- Fleet coordination
- Data logging (2TB storage)

## Robot Arm Platform

Desktop manipulation robot:

```
+------------------+
|    Intel NUC     |
+------------------+
        |
+------------------+
| 6-DOF Robot Arm  |  <-- MoveIt2 control
+------------------+
        |
+------------------+
|  Depth Camera    |  <-- Object detection
+------------------+
```

## Development Server

Train models and develop code for Pi deployment:

- Faster iteration than Colab
- Local Jupyter notebooks
- Docker containers
- CI/CD for robot code
