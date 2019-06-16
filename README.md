# Orocos KDL Ubuntu Preempt Benchmark
- Orocos KDL Inverse Dynamics for 6DOF Robot Arm Calcualation.
  - Orocos KDL - Euler–Lagrange Equation.
- Measure Execution time to compare:
  - No Preempt - Preempt - Preempt with thread and memory control.
  - Orocos KDL - Matlab compare.

- Copy code from Source_code to main.cpp
```
mkdir build
cd build
cmake ..
make
./main
```
