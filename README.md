# 2D Least Squares based bearing-only SLAM 

This is a small project developed as final work for the [Probabilistic Robotics](https://sites.google.com/dis.uniroma1.it/probabilistic-robotics/home) course offered in the Master in Artificial Intelligence and Robotics held by Sapienza University of Rome. Given as input: 

* 2D bearing-only labeled observations 
* odometry

The goal of the project is to recover (using Least Squares as a tool):
 
* trajectory 
* map

Being the observations labeled, no data association is addressed here.

## Folders

The material in the repository is organized as follows:

- [dataset](https://github.com/pavelVice/PR_project/tree/master/dataset): starting material for the project
- [tools](https://github.com/pavelVice/PR_project/tree/master/tools): g2o and geometry helpers
- [inspect dataset](https://github.com/pavelVice/PR_project/tree/master/inspect_dataset): visualization of the ground truth and measurements contained in the dataset
- [2DLSBasedBearingOnlySLAM](https://github.com/pavelVice/PR_project/tree/master/2DLSBasedBearingOnlySLAM): 2D least squares based SLAM using bearing measurements only 
- [figures](https://github.com/pavelVice/PR_project/tree/master/figures): figures described in this README

## Inspect the dataset

The input [dataset](https://github.com/pavelVice/PR_project/tree/master/dataset) for the project consists in a .g2o file containing both the ground truth (landmarks and poses) and the observations (labeled bearings and odometry). 

At first, the input dataset is loaded in Octave using the [g2o wrapper](https://gitlab.com/grisetti/probabilistic_robotics_2017_18/tree/master/applications/octave/tools/g2o_wrapper) made available during the course and then inspected by plotting the ground truth: landmark positions (blue), robot trajectory (red) and ideal observatons (green), i.e. the observations obtained by linking each pose-landmark couple appearing in the labeled bearing observations.
 
![Alt text](./figures/fig1.png?raw=true "")

As you can easily notice, some observations are meaningless for the problem at issue (for instance, the bearings of the landamarks observed by one single robot pose): they will be discarded. 

Even the quality of the observations is inspected by comparing, for a few poses, the real pose (red) and the corresponding one in the odometry trajectory (green), as well as the ideal bearing observations (blue) and the real ones (black) for the pose at issue.

![Alt text](./figures/fig2.png?raw=true "")

![Alt text](./figures/fig3.png?raw=true "")

As expected, at the beginning of the trajectory (top) they overlap, apart from the noise, while with the evolution of the trajectory (es. in the middle, bottom) the error grows substantially.

## Development steps

The starting point for the project is the [Total Least Squares](https://gitlab.com/grisetti/probabilistic_robotics_2018_19/tree/master/applications/octave/26_total_least_squares) octave application developed during the lectures, which implements a least squares system for pose-landmark, pose-landmark-projection and pose-pose constraints on top of a synthetic 3D environment, assuming ideal observations (each landmark position is observed from each pose, without noise) and deriving a good initial guess for the solver via small perturbations of the ground truth.

At first, the application is adapted to a 2D scenario. As a first attempt, only the pose-landmark constraints are considered and modified to use bearings as observations. In a synthetic 2D scenario, keeping the ideal observations (each landmark bearing is observed from each pose, without noise) and the good initial guess, the system seems to converge to a bad solution. 

![Alt text](./figures/fig4.png?raw=true "")

Due to the bearing-only constraint, the system is scale-free and therefore one landmark need to be fixed in order to get a better solution.

![Alt text](./figures/fig5.png?raw=true "")
  
The synthetic 2D scenario is therefore replaced by the input ground truth (landmark positions and robot poses). Initializing the poses with the measured odometry trajectory, while keeping the ideal initialization for the landmarks (small perturbation of the ground truth) as well as the ideal measurements (each landmark bearing is observed from each pose, without noise), the solution is still good.

![Alt text](./figures/fig6.png?raw=true "")

The same holds if you randomly select only a fraction of the ideal observations (noise-free) so that they are almost as many as the real ones provided in the input file.

![Alt text](./figures/fig7.png?raw=true "")

But if you select as ideal observations (noise-free) exactly the ones provided in the input dataset (i.e. those visualized when inspecting the ground truth), the system explodes.

![Alt text](./figures/fig8.png?raw=true "")


This suggests the introduction of the pose-pose constraints (from the odometry) to make the system more robust. After such a modification, the system converges even using as ideal observations (noise-free) exactly the ones provided in the input dataset, even if some landmarks (e.g. right-bottom or left-bottom) are localized much worse than the others.

![Alt text](./figures/fig9.png?raw=true "")

Looking at the ground truth initial visualization, you can notice how the bad-localized landmarks are indeed those observed by one single pose, or more in general those for which the observed global bearings are too similar each other and therefore convey meaningless information. By filtering them out (in particular, by filtering out the landmarks having a variance of the observed global bearings less than 0.1), the solution improves.  

![Alt text](./figures/fig10.png?raw=true "")

The last step for the project is to remove ideality from landmark initialization and observations. The ideal observations (noise-free) are simply replaced by the bearings provided in the input dataset. The initialization of a landmark instead works as follows: each couple of bearing observations related to the landmark is triangulated in order to get an estimate of the landmark position (based on the given couple of measurements) and then the landmark is initialized to the mean of all the estimates. The mechanism is shown in the following figure, a zoom on the right part of the trajectory showing how the bearings (green) starting from the odometry guesses (red) are intersected and an initial guess for a the landmark at issue (magenta) is recovered as a mean of all the intersections (black). The initial guess for the landmark is in this case quite distant from the true position because of the fact that the odometry guess is quite far from the real trajectory, but it is the best initial guess you can retrieve from the available observations.

![Alt text](./figures/fig11.png?raw=true "")

Using the real bearings and the aforementioned initialization procedure for the landmarks, the landmark initialization is affected by much more noise, but the trajectory and map found still represent a good result.

![Alt text](./figures/fig12.png?raw=true "")

## How to run

### Inspect the dataset:
    cd inspect_dataset
    octave-cli 
    inspect_dataset 

Note: close the figures in order to proceed.


###Perform bearing-only SLAM:
    cd 2DLSBasedBearingOnlySLAM
    octave-cli 
    main

