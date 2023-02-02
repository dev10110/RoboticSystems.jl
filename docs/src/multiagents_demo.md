# Multiagent Simulations

`RoboticSystems.jl` makes it very easy to define multiagent simulations. This is achieved through `ComponentArrays` and `DiffEq` working well together. 

We will simulate a quadrotor flying in a circle, and a second quadrotor trying to circle around it. 


## Define the quadrotors
```@example MA

using ComponentArrays, Plots, LinearAlgebra, Parameters
using RoboticSystems
RS = RoboticSystems

gr()

# define the quadrotor parameters
quad1_p = ComponentArray(
  RS.quadrotor_parameters;
  mass=1.5) |> RS.initialize_quad_params

quad2_p = ComponentArray(
  RS.quadrotor_parameters;
  mass=0.8) |> RS.initialize_quad_params


quad1_ic = ComponentArray(
    x = zeros(3),
    v = zeros(3),
    R = 1.0(I(3)) |> collect,
    Ω = zeros(3),
    ω = zeros(4)
);

quad2_ic = ComponentArray(
    x = [1,0,0.], # this one starts at a different location 
    v = zeros(3),
    R = 1.0(I(3)) |> collect,
    Ω = zeros(3),
    ω = zeros(4)
);

##Plot the quads
plot()
RS.plot_quad!(quad1_ic, quad1_p)
RS.plot_quad!(quad2_ic, quad2_p)
RS.plot_iso3d!()
```


## Define the controller for the first quad

```@example MA

cont1_p = ComponentArray(
    kx = 1.0,
    kv = 2.0,
    kR = 0.35,
    kΩ = 0.15,
    m = quad1_p.mass,
    J = quad1_p.J,
    invG = quad1_p.invG,
    g = 9.81
)

function controller1(state, cont_p, t)

    ## Circular Trajectory
    w = 2*π/10
    xd = [sin(w*t), sin(w*t+π/2), 1]
    vd = [w*sin(w*t + π/2), w*sin(w*t + π/2 + π/2), 0]
    ad = [w^2*sin(w*t + 2*π/2), w^2*sin(w*t + π/2 + 2*π/2), 0]
    jd = [w^3*sin(w*t + 3*π/2), w^3*sin(w*t + π/2 + 3*π/2), 0]
    sd = [w^4*sin(w*t + 4*π/2), w^4*sin(w*t + π/2 + 4*π/2), 0]

    ## define the desired yaw, yaw rate, and yaw acceleration at current time
    ψ = 0.0
    ψd = 0.0
    ψdd = 0.0

    ## convert into target quadrotor state,  using differential flatness
    target = RS.flat_state_to_quad_state(xd, vd, ad, jd, sd, ψ, ψd, ψdd)

    ## determine required thrust and moments, using geometric controller
    fM = RS.geometric_controller(state, target..., cont_p)

    ## determine rotation speed of each motor
    control = RS.fM_to_ω(fM, cont_p)

end

cont2_p = ComponentArray(
    kx = 1.0,
    kv = 2.0,
    kR = 0.35,
    kΩ = 0.15,
    m = quad2_p.mass,
    J = quad2_p.J,
    invG = quad2_p.invG,
    g = 9.81
)

function controller2(state, cont_p, quad1_pos, t)

    ## Maintain a position relative to quad 1
    xd = quad1_pos + [0; 0 ; 1.]
    vd = zeros(3) 
    ad = zeros(3) 
    jd = zeros(3) 
    sd = zeros(3)

    ## define the desired yaw, yaw rate, and yaw acceleration at current time
    ψ = 0.0
    ψd = 0.0
    ψdd = 0.0

    ## convert into target quadrotor state,  using differential flatness
    target = RS.flat_state_to_quad_state(xd, vd, ad, jd, sd, ψ, ψd, ψdd)

    ## determine required thrust and moments, using geometric controller
    fM = RS.geometric_controller(state, target..., cont_p)

    ## determine rotation speed of each motor
    control = RS.fM_to_ω(fM, cont_p)

end


function closed_loop!(D, state, params, t)

    @unpack quad1, quad2 = state
    @unpack quad1_p, quad2_p, cont1_p, cont2_p = params

    u1 = controller1(quad1, cont1_p, t)
    u2 = controller2(quad2, cont2_p, quad1.x, t)

    # update the dynamics
    RS.quadrotor!(D.quad1, quad1, quad1_p, u1, t)
    RS.quadrotor!(D.quad2, quad2, quad2_p, u2, t)

    return
end

```

## Perform the Simulation

```@example MA
using DifferentialEquations

tspan = (0, 25.0)

# combine the states
full_state_ic = ComponentArray(
  quad1 = quad1_ic,
  quad2 = quad2_ic
)

# combine all the parameters
params = ComponentArray(
  quad1_p = quad1_p,
  quad2_p = quad2_p,
  cont1_p = cont1_p,
  cont2_p = cont2_p,
)


## try the closed loop function
D_ = similar(full_state_ic)
closed_loop!(D_, full_state_ic, params, 0.0)

# define the ODE problem
prob = ODEProblem(closed_loop!, full_state_ic, tspan, params)

## Solve the problem
sol = solve(prob, Tsit5())


## Plot the trajectory
plot()
RS.plot_quad_traj!(sol, quad1_p; tspan=tspan, prefix="quad1")
RS.plot_quad_traj!(sol, quad2_p; tspan=tspan, prefix="quad2")
RS.plot_iso3d!()
RS.plot_project3d!()
```

```@example MA

anim = @animate for tm = range(sol.t[1], sol.t[end], 200)

    tspan=(sol.t[1], tm)

    plot()

    RS.plot_quad_traj!(sol, quad1_p; tspan=tspan, prefix="quad1")
    RS.plot_quad_traj!(sol, quad2_p; tspan=tspan, prefix="quad2")

    plot!(xlims=(-1.5, 1.5), ylims=(-1.5, 1.5), zlims=(0, 2.5))

    # rotate camera
    plot!(camera = (360 * ((tm - sol.t[1]) / (sol.t[end] - sol.t[1])), 30))
    RS.plot_iso3d!()
    RS.plot_project3d!()

end;

gif(anim)
```
