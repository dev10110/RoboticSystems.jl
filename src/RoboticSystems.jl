module RoboticSystems


using LinearAlgebra
using ComponentArrays
using Parameters: @unpack
using Plots
using Printf
using Rotations
using StaticArrays



include("types.jl")
include("utils.jl")
include("trajectories.jl")
include("plotting.jl")
include("quadrotors/quadrotors.jl")

end
