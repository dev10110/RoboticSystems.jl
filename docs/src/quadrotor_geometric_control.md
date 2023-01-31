# Quadrotor Quickstart

This is a walkthrough example on how to use this library. The whole point is to make it easily extensible, and to allow users to easily swap in different controllers and robotic systems. 

We will simulate a quadrotor executing a 3D Lissajous trajectory. In the end, we will produce the following animation:

![](quad_traj.gif)

## Define the quadrotor


```@example quad
using ComponentArrays, Plots, LinearAlgebra, Parameters
using RoboticSystems
RS = RoboticSystems

gr()   # uncomment for fast (but non-interactive) 3d plots
# plotly() # uncomment for interactive 3d plots. Does not support animations.

## Define parameters of the quad
# start with default parameters, and modify some values
quad_p = ComponentArray(
  RS.quadrotor_parameters; 
  mass=1.5,
  k_drag_f =  0.02
) |> RS.initialize_quad_params

## Define the quad's initial condition
quad_ic = ComponentArray(
    x = zeros(3),
    v = zeros(3),
    R = 1.0(I(3)) |> collect, 
    Ω = zeros(3),
    ω = 2*RS.hover_ω(quad_p)
);

##Plot the quadrotor
plot()
RS.plot_quad!(quad_ic, quad_p)
RS.plot_triad!(quad_ic)
RS.plot_iso3d!()
```

Notes:
- the initial state of the quadrotor is expressed using a `ComponentArray`. This is an incredible library, and is the keytool to allow us to compose various equations together.
- notice the `|>` when we constructed `quad_p`: we passed `quad_p` through `initialize_quad_params`.  This is necessary to initialize (and invert) some commonly used matrices, to avoid runtime cost
- `RoboticSystems` provides some convenient plotting tools.
  - `RS.plot_quad!` plots a quadrotor
  - `RS.plot_triad` plots the XYZ axes of a rotation matrix as red (x) green (y) and blue (z) lines. By default, we only use ENU frames, and `quad_ic.R` is a rotation matrix from the quadrotor-body fixed frame **to** to inertial frame.  
  - `RS.plot_iso3d!` adjusts x,y,zlims to make the plot look like `aspect_ratio=1` 


## Define the closed-loop function

Next, we need to define the closed-loop dynamics, by using a feedback controller. 
The controller used in this example is a geometric controller that uses differential flatness to track a path. 

```@example quad; output = false
## Define a trajectory
# a Lissajous trajectory
trajectory_params = ComponentArray(
    A = [1, 1, 0.2],
    ω = [5/8,4/8,6/8],
    ϕ = [π/2, 0, 0],
    off = [0,0,1.]
)

## Define the controller

controller_params = ComponentArray(
    kx = 1.0, 
    kv = 2.0,
    kR = 0.35,
    kΩ = 0.15, 
    m = 1.5,
    J = quad_p.J, # this is cheating, but is convenient,
    invG = quad_p.invG, # this is also cheating...
    g = 9.81,
    trajectory_params = trajectory_params
)

function controller(state, controller_params, t)

    @unpack trajectory_params = controller_params

    ## define the desired position, velocity, acceleration, jerk, snap at the current time
    xd = RS.lissajous(t, trajectory_params)
    vd = RS.lissajous(t, trajectory_params, 1)
    ad = RS.lissajous(t, trajectory_params, 2)
    jd = RS.lissajous(t, trajectory_params, 3)
    sd = RS.lissajous(t, trajectory_params, 4)

    ## define the desired yaw, yaw rate, and yaw acceleration at current time
    ψ = 3.0*t/8
    ψd = 3.0/8
    ψdd = 0.0

    ## convert into target quadrotor state,  using differential flatness
    target = RS.flat_state_to_quad_state(xd, vd, ad, jd, sd, ψ, ψd, ψdd)

    ## determine required thrust and moments, using geometric controller
    fM = RS.geometric_controller(state, target..., controller_params)
    # returns a vector [f, M[1], M[2], M[3]]

    ## determine rotation speed of each motor
    control = RS.fM_to_ω(fM, controller_params)
    # returns the desired ω of each motor

end


## define the closed loop dynamics
function cl_geometric_quad!(D, state, params, t)
    
    @unpack quad_params, controller_params = params

    u = controller(state, controller_params, t)

    # define the external wind speed (constant, for simplicity)
    w_ext = [1.0, 0, 0.]

    # determine the rate of change of state based on control input and other forces/torques
    RS.quadrotor!(D, state, quad_params, u, t; wind_ext=w_ext)

end
```


You can imagine more complicated wind speeds can be modelled. Additional external forces or torques can be specified in `RS.quadrotor!`.  



## Perform the Simulation

We can use the excellent library `DifferentialEquations.jl` to achieve high performance and extensible ODE solving. `RoboticSystems.jl` doesnt __require__ you to use `DiffEq.jl`, but it is written in a way that makes it easy to use `DiffEq.jl`. 



```@example quad; output=false
using DifferentialEquations

tspan = (0, 100.0)

# combine all the parameters
params = ComponentArray(
  quad_params = quad_p,
  controller_params = controller_params
)

# define the ODE problem
prob = ODEProblem(cl_geometric_quad!, quad_ic, tspan, params)

## Solve the problem
sol = solve(prob, Tsit5())

```
All of the arguments (e.g. `restol, abstol`) that you normally pass into `ODEProblem` or `solve` can be used. Refer to the `DifferentialEquations.jl` documentation for more details. 


## Plot/Animate the solution

```@example quad
plotly()

plot()
RS.plot_quad_traj!(sol, quad_p; tspan=tspan)

plot!(xlims=(-1.5, 1.5), ylims=(-1.5, 1.5), zlims=(0, 1.5))

RS.plot_iso3d!()
RS.plot_project3d!()
```

Next, we can use `Plots.jl`'s animation tools to construct a simple animation. 

```@example quad
gr()

anim = @animate for tm = range(sol.t[1], sol.t[end], 200)
    
    tspan=(sol.t[1], tm)
    
    plot()
    RS.plot_quad_traj!(sol, quad_p; tspan=tspan)
    
    plot!(xlims=(-1.5, 1.5), ylims=(-1.5, 1.5), zlims=(0, 1.5))
    
    # rotate camera
    plot!(camera = (360 * ((tm - sol.t[1]) / (sol.t[end] - sol.t[1])), 30))
    plot!(axis = ([], false), xlabel="", ylabel="", zlabel="")
    RS.plot_iso3d!()
    RS.plot_project3d!()

end;

gif(anim)
```

To plot any additional plot, you can use `sol` as a function, for example:
```@example quad
gr()
plot(t-> RS.roll(sol(t).R) * 180 / π, tspan..., label="sim")
plot!(xlabel="time (s)", ylabel="roll (degrees)")
```
