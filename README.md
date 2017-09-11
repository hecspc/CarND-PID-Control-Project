# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---


## PID Controller

Given a trajectory we want the car to follow and `cte` is the cross track error given the position of the car, our goal is to control the car to follow the desire track smoothly.

To achieve this goal we make use of the PID controller. The PID controller has 3 components:


### P Proportional (gain)
We multiply the `cte` by a negative proportional constant `Kp`. This coefficient `Kp` represents the band over the controller's output is proportional to the error of the system. The issue is that the controll will always overshoot the desired value, so there will be always an error between the desired value and the current value which will oscilate.


### I Integral components (reset)
This component will average the `cte` over a period of time and multiplied by a component `Ki`. It represents the steady error of the system and will remove the error from the mesaurements. In our case this could be due to a misaligments of the wheels, for example.

### D Differential components (rate)
This component calculates the rate of change the `cte` with respect of time and multiplied by a component `Kd`. This term determines the system's response to a change or disturbance. The larger the component, the more rapidly the controller will respond to the changes. In our case how fast the car is goint to recover.

## Tuning PID controller coefficients

To tune the coefficients `Kp`, `Ki` and `Kd` I  have used the __Twiddle__ algorithm implemented in the lines 31-76 in `main.cpp` file.

The twiddle algorithm adjusts the coefficients of the PID controller trying to find the best overall error.

By default Twiddle is off, but to enable the variable `twiddle_on_` in the line 31 should be set to `true`;

After running the simulation several times during a long period of time, I have found this values for the smoothest experience:

```
pid_steer.Init(0.265, 0.0, 3);
```

It is noticeable that according to the thw Twiddle algorithm the `Ki` component, which measures the systematic errors from the system, is zero, meaning we can assume there are perfect steering. Given this is a simulation, it makes sense.






## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

