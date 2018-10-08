# P6: Extended Kalman Filter Project

The project includes the following files

* CMakeLists.txt
* src folder
* notes.txt (this file)

## Description

After downloading the project and compare the TODOs with the Quizzes, it was really useful the [QA session](https://youtu.be/J7WK9gEUltM).

Once the program is running with the basics, I continued with the Tips and Tricks section [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/cb31243c-3983-4a13-8799-5db4b0488b5e).

* angle normalization useful tips from [here](https://discussions.udacity.com/t/ekf-radar-causes-rmse-to-go-through-the-roof/243944/5)
* Noise issues considerations from [here](https://discussions.udacity.com/t/methodology-for-improving-ekf-rmse/381066/12)
* Initialization issues got fixed using tips from [here](https://discussions.udacity.com/t/terrible-accuracy-with-lidar-and-even-worse-with-radar/441139/2)
* Prevent division by zero or very low values

## Test
I got errors when switching from dataset 1 to dataset 2. I found these threads with information about it:

* [stuck-with-x--nan](https://discussions.udacity.com/t/stuck-with-x--nan-and-p--nan/323082/10)
* [sim-linux-not-working-for-dataset2](https://discussions.udacity.com/t/running-term2-sim-x86-64-term2-sim-linux-not-working-for-dataset2-and-output/246868)

My solution was to:

1. Compile the program as detailed on the repository.
2. Run `./ExtendedKF `
3. select the detaset and test it.
4. When switching between datasets, please rerun the ExtendedKF program.

## Results

Dataset 1

RMSE
x:0.0973
y:0.0855
VX:0.4513
VY:0.4399

Dataset 2

RMSE
x:0.0734
y:0.0979
VX:0.3782
VY:0.5026
