using HW1
using Test
using LinearAlgebra
using Statistics
using JLD2
using FileIO
using RobotZoo
using RobotDynamics
using StaticArrays

nb = 1:3
HW1.notebook(nb)

##
nb = nb isa Integer ? UnitRange(nb,nb) : nb
1 ∈ nb && include("q1.jl")
2 ∈ nb && include("q2.jl")
3 ∈ nb && include("q3.jl")
