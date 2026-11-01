# 6-DOF Robot Arm: Kinematics and Simulation in MATLAB

Final project for an *Introduction to Robotics* course. A 6-DOF pen-holding robot arm is modelled with
Denavit-Hartenberg (DH) parameters. The project includes an interactive 3D simulator built from SolidWorks STL
models, a numerical inverse-kinematics solver that makes the pen tip trace a tilted ellipse in space, and an
analytical solver that returns all 8 inverse-kinematics solutions for a point.

## What's inside

| File | What it does |
|---|---|
| `robot_gui_ellipse.m` | Interactive 3D simulator: six joint sliders, live coordinate frames, and an **Animate Red Ellipse** button |
| `inverse_kinematics_8sol.m` | Analytical inverse kinematics: enter a target point and get all 8 joint-angle solutions |
| `models/` | STL parts of the arm exported from SolidWorks (base, links, end link, pen holder) |

## Simulator

`robot_gui_ellipse` loads the STL parts and moves them with DH transforms. The six sliders set the joint angles,
and the pen tip is marked with a black dot. The frame of every link is drawn with its X (red), Y (green) and
Z (blue) axes.

**Animate Red Ellipse** makes the pen tip follow a tilted ellipse (101 points, semi-axes 150 and 100 mm,
centre at (180, 0, 300) mm, rotated by Z 25°, Y 15°, X 10°). For each point the joint angles are found numerically
with `fminsearch`. This is repeated for 8 different starting guesses, which gives the 8 solution families.
The result of each run is saved to the MATLAB workspace as a table named `Solution_Table_1` ... `Solution_Table_8`,
with the pen position and the joint angles of every point.

## Analytical inverse kinematics

`inverse_kinematics_8sol` solves the arm in closed form for a pen held perpendicular to the page:

1. Find the wrist centre by moving back from the target along the pen.
2. Check that the point is within reach (otherwise it reports `Point out of reach!`).
3. Solve the base angle (2 solutions), the elbow (2 solutions: up and down) and the shoulder (law of cosines).
4. Solve the wrist (2 solutions: normal and flipped by 180°).

2 × 2 × 2 = **8 solutions**, printed in a table with the joint names.

Robot data used by the solver: upper arm and forearm 125 mm each, tool offset 37.5 mm, pen length 100 mm.

## Run it

Requirements: MATLAB R2018b or newer (for `stlread`). No extra toolboxes. Run from the repository root so that
`models/` is found:

```matlab
robot_gui_ellipse            % opens the simulator
inverse_kinematics_8sol      % asks for X, Y, Z and prints 8 solutions
```

Angles are in degrees and lengths in millimetres. If an STL file is missing, the simulator falls back to simple
placeholder shapes instead of crashing.

## Notes

- Some code comments are in Hebrew.
- Course material (assignment text, report) is not included.

## License

MIT, see [LICENSE](LICENSE).
