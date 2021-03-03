# Optimal Control 16-745 HW1: Due Thursday, February 18
![CI](https://github.com/Optimal-Control-16-745/HW1/workflows/CI/badge.svg)

In this homework we'll be exploring topics in dynamics and optimization. Here's an overview of the problems:
1. Implement an implicit integrator and compare performance against an explicit method
2. Find an equilibrium state to balance a quadruped on one leg using Newton's method and unconstrained optimization
3. Simulate a falling brick by solving a quadratic program. Implement your own augmented Lagrangian QP solver

## Getting Started
All the homeworks are distributed as Jupyter notebooks. Follow these instructions to get everything set up.

1. Install Julia. Download v1.5.3 from the [Julia website](https://julialang.org/downloads/). Extract the binaries onto a stable location on your computer, and add the `/bin` folder to your system path.
2. Clone this repo, and open a terminal in the root directory
2. Start a Julia REPL in your terminal using `julia`. This should work as long as the binaries are on your system path.
3. Install the [IJulia](https://github.com/JuliaLang/IJulia.jl) using the Julia package manager. In the REPL, enter the package manager using `]`, then enter `add IJulia` to add it to your system path.
4. In the REPL (hit backspace to exit the package manager), enter `using IJulia`
5. Launch the notebook using `notebook()` or `jupyterlab()`

## Submitting your Homework
Once you have pushed your final changes to GitHub, go to Github and select "Create a new release" on the right-hand side.
Tag your release `v1.0` and set the title to be "Submission." If you have any comments about your release, such as 
explaining any code that may not be working, or want to provide some details about a unique approach, feel free to include
them in the description.

The autograder will run once your release is submitted. If your tests don't pass, don't worry. We'll still run your code 
locally and give you credit for what runs on our machines, and for what you've attempted to complete.

## Running the Autograder
You don't have to wait until your final submission to run the autograder. 
To run the autograder locally, boot up a REPL in the root directory of this repository. Activate the current environment in the package manager (i.e. after entering `]`) using
`activate .`. Then run the test suite (still in the package manager) enter `test HW1` and it will launch the testing suite. You will see a printout of the progress of the tests.

For more information on the autograder, see [this file](https://github.com/Optimal-Control-16-745/JuliaIntro/blob/main/docs/Autograding.md).

### Testing Questions Individually
To run the tests for an individual question, follow these steps:
1. Edit the `nb` variable in `test/runtests.jl` to be the question(s) you want. It can be either a range (e.g. `2:3`) or an integer (e.g. `2`).
2. Run the test script. You can do this any of the following ways. All commands assume you have a terminal at the root directory of the repository:
    * Use package manager to run full test suite. In the REPL package manager, run `activate .`, then `test HW1`.
    * Run from the REPL. In the REPL, activate the current project via `] activate .`, then in the REPL (hit backspace to exit the package manager) run the test file using `include("test/runtests.jl")`.
    * Run directly from the command line. Enter the following command in the terminal: `julia --project=. test/runtests.jl`

Remember to change the `nb` variable back to `1:3` before you submit your final version.
We'll try to make this process easier in future homeworks.

## Questions / Issues
If you have questions as you work through the homework, please post to the `hw1` channel on Slack, or sign up for office hours.

