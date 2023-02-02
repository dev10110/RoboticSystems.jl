var documenterSearchIndex = {"docs":
[{"location":"multiagents_demo/#Multiagent-Simulations","page":"Multiagent","title":"Multiagent Simulations","text":"","category":"section"},{"location":"multiagents_demo/","page":"Multiagent","title":"Multiagent","text":"RoboticSystems.jl makes it very easy to define multiagent simulations. This is achieved through ComponentArrays and DiffEq working well together. ","category":"page"},{"location":"multiagents_demo/","page":"Multiagent","title":"Multiagent","text":"We will simulate a quadrotor flying in a circle, and a second quadrotor trying to circle around it. ","category":"page"},{"location":"multiagents_demo/#Define-the-quadrotors","page":"Multiagent","title":"Define the quadrotors","text":"","category":"section"},{"location":"multiagents_demo/","page":"Multiagent","title":"Multiagent","text":"\nusing ComponentArrays, Plots, LinearAlgebra, Parameters\nusing RoboticSystems\nRS = RoboticSystems\n\ngr()\n\n# define the quadrotor parameters\nquad1_p = ComponentArray(\n  RS.quadrotor_parameters;\n  mass=1.5) |> RS.initialize_quad_params\n\nquad2_p = ComponentArray(\n  RS.quadrotor_parameters;\n  mass=0.8) |> RS.initialize_quad_params\n\n\nquad1_ic = ComponentArray(\n    x = zeros(3),\n    v = zeros(3),\n    R = 1.0(I(3)) |> collect,\n    Ω = zeros(3),\n    ω = zeros(4)\n);\n\nquad2_ic = ComponentArray(\n    x = [1,0,0.], # this one starts at a different location \n    v = zeros(3),\n    R = 1.0(I(3)) |> collect,\n    Ω = zeros(3),\n    ω = zeros(4)\n);\n\n##Plot the quads\nplot()\nRS.plot_quad!(quad1_ic, quad1_p)\nRS.plot_quad!(quad2_ic, quad2_p)\nRS.plot_iso3d!()","category":"page"},{"location":"multiagents_demo/#Define-the-controller-for-the-first-quad","page":"Multiagent","title":"Define the controller for the first quad","text":"","category":"section"},{"location":"multiagents_demo/","page":"Multiagent","title":"Multiagent","text":"\ncont1_p = ComponentArray(\n    kx = 1.0,\n    kv = 2.0,\n    kR = 0.35,\n    kΩ = 0.15,\n    m = quad1_p.mass,\n    J = quad1_p.J,\n    invG = quad1_p.invG,\n    g = 9.81\n)\n\nfunction controller1(state, cont_p, t)\n\n    ## Circular Trajectory\n    w = 2*π/10\n    xd = [sin(w*t), sin(w*t+π/2), 1]\n    vd = [w*sin(w*t + π/2), w*sin(w*t + π/2 + π/2), 0]\n    ad = [w^2*sin(w*t + 2*π/2), w^2*sin(w*t + π/2 + 2*π/2), 0]\n    jd = [w^3*sin(w*t + 3*π/2), w^3*sin(w*t + π/2 + 3*π/2), 0]\n    sd = [w^4*sin(w*t + 4*π/2), w^4*sin(w*t + π/2 + 4*π/2), 0]\n\n    ## define the desired yaw, yaw rate, and yaw acceleration at current time\n    ψ = 0.0\n    ψd = 0.0\n    ψdd = 0.0\n\n    ## convert into target quadrotor state,  using differential flatness\n    target = RS.flat_state_to_quad_state(xd, vd, ad, jd, sd, ψ, ψd, ψdd)\n\n    ## determine required thrust and moments, using geometric controller\n    fM = RS.geometric_controller(state, target..., cont_p)\n\n    ## determine rotation speed of each motor\n    control = RS.fM_to_ω(fM, cont_p)\n\nend\n\ncont2_p = ComponentArray(\n    kx = 1.0,\n    kv = 2.0,\n    kR = 0.35,\n    kΩ = 0.15,\n    m = quad2_p.mass,\n    J = quad2_p.J,\n    invG = quad2_p.invG,\n    g = 9.81\n)\n\nfunction controller2(state, cont_p, quad1_pos, t)\n\n    ## Maintain a position relative to quad 1\n    xd = quad1_pos + [0; 0 ; 1.]\n    vd = zeros(3) \n    ad = zeros(3) \n    jd = zeros(3) \n    sd = zeros(3)\n\n    ## define the desired yaw, yaw rate, and yaw acceleration at current time\n    ψ = 0.0\n    ψd = 0.0\n    ψdd = 0.0\n\n    ## convert into target quadrotor state,  using differential flatness\n    target = RS.flat_state_to_quad_state(xd, vd, ad, jd, sd, ψ, ψd, ψdd)\n\n    ## determine required thrust and moments, using geometric controller\n    fM = RS.geometric_controller(state, target..., cont_p)\n\n    ## determine rotation speed of each motor\n    control = RS.fM_to_ω(fM, cont_p)\n\nend\n\n\nfunction closed_loop!(D, state, params, t)\n\n    @unpack quad1, quad2 = state\n    @unpack quad1_p, quad2_p, cont1_p, cont2_p = params\n\n    u1 = controller1(quad1, cont1_p, t)\n    u2 = controller2(quad2, cont2_p, quad1.x, t)\n\n    # update the dynamics\n    RS.quadrotor!(D.quad1, quad1, quad1_p, u1, t)\n    RS.quadrotor!(D.quad2, quad2, quad2_p, u2, t)\n\n    return\nend\n","category":"page"},{"location":"multiagents_demo/#Perform-the-Simulation","page":"Multiagent","title":"Perform the Simulation","text":"","category":"section"},{"location":"multiagents_demo/","page":"Multiagent","title":"Multiagent","text":"using DifferentialEquations\n\ntspan = (0, 25.0)\n\n# combine the states\nfull_state_ic = ComponentArray(\n  quad1 = quad1_ic,\n  quad2 = quad2_ic\n)\n\n# combine all the parameters\nparams = ComponentArray(\n  quad1_p = quad1_p,\n  quad2_p = quad2_p,\n  cont1_p = cont1_p,\n  cont2_p = cont2_p,\n)\n\n\n## try the closed loop function\nD_ = similar(full_state_ic)\nclosed_loop!(D_, full_state_ic, params, 0.0)\n\n# define the ODE problem\nprob = ODEProblem(closed_loop!, full_state_ic, tspan, params)\n\n## Solve the problem\nsol = solve(prob, Tsit5())\n\n\n## Plot the trajectory\nplot()\nRS.plot_quad_traj!(sol, quad1_p; tspan=tspan, prefix=\"quad1\")\nRS.plot_quad_traj!(sol, quad2_p; tspan=tspan, prefix=\"quad2\")\nRS.plot_iso3d!()\nRS.plot_project3d!()","category":"page"},{"location":"multiagents_demo/","page":"Multiagent","title":"Multiagent","text":"\nanim = @animate for tm = range(sol.t[1], sol.t[end], 200)\n\n    tspan=(sol.t[1], tm)\n\n    plot()\n\n    RS.plot_quad_traj!(sol, quad1_p; tspan=tspan, prefix=\"quad1\")\n    RS.plot_quad_traj!(sol, quad2_p; tspan=tspan, prefix=\"quad2\")\n\n    plot!(xlims=(-1.5, 1.5), ylims=(-1.5, 1.5), zlims=(0, 2.5))\n\n    # rotate camera\n    plot!(camera = (360 * ((tm - sol.t[1]) / (sol.t[end] - sol.t[1])), 30))\n    RS.plot_iso3d!()\n    RS.plot_project3d!()\n\nend;\n\ngif(anim)","category":"page"},{"location":"reference/#Reference","page":"Reference","title":"Reference","text":"","category":"section"},{"location":"reference/","page":"Reference","title":"Reference","text":"Modules = [RoboticSystems]","category":"page"},{"location":"reference/#RoboticSystems.flat_state_to_quad_state-Union{Tuple{ComponentArrays.ComponentVector{F}}, Tuple{F}, Tuple{ComponentArrays.ComponentVector{F}, F}} where F","page":"Reference","title":"RoboticSystems.flat_state_to_quad_state","text":"Based on \"Accurate Tracking of Aggressive Quadrotor Trajectories using Incremental Nonlinear Dynamic Inversion and Differential Flatness\" by Ezra Tal, Sertac Karaman\n\n\n\n\n\n","category":"method"},{"location":"","page":"Home","title":"Home","text":"CurrentModule = RoboticSystems","category":"page"},{"location":"#RoboticSystems","page":"Home","title":"RoboticSystems","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"Documentation for RoboticSystems.","category":"page"},{"location":"","page":"Home","title":"Home","text":"","category":"page"},{"location":"#Quickstart:","page":"Home","title":"Quickstart:","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"See the examples at:","category":"page"},{"location":"","page":"Home","title":"Home","text":"","category":"page"},{"location":"quadrotor_geometric_control/#Quadrotor-Quickstart","page":"Quadrotor QuickStart","title":"Quadrotor Quickstart","text":"","category":"section"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"This is a walkthrough example on how to use this library. The whole point is to make it easily extensible, and to allow users to easily swap in different controllers and robotic systems. ","category":"page"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"We will simulate a quadrotor executing a 3D Lissajous trajectory. In the end, we will produce the following animation:","category":"page"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"(Image: )","category":"page"},{"location":"quadrotor_geometric_control/#Define-the-quadrotor","page":"Quadrotor QuickStart","title":"Define the quadrotor","text":"","category":"section"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"using ComponentArrays, Plots, LinearAlgebra, Parameters\nusing RoboticSystems\nRS = RoboticSystems\n\ngr()   # uncomment for fast (but non-interactive) 3d plots\n# plotly() # uncomment for interactive 3d plots. Does not support animations.\n\n## Define parameters of the quad\n# start with default parameters, and modify some values\nquad_p = ComponentArray(\n  RS.quadrotor_parameters; \n  mass=1.5,\n  k_drag_f =  0.02\n) |> RS.initialize_quad_params\n\n## Define the quad's initial condition\nquad_ic = ComponentArray(\n    x = zeros(3),\n    v = zeros(3),\n    R = 1.0(I(3)) |> collect, \n    Ω = zeros(3),\n    ω = 2*RS.hover_ω(quad_p)\n);\n\n##Plot the quadrotor\nplot()\nRS.plot_quad!(quad_ic, quad_p)\nRS.plot_triad!(quad_ic)\nRS.plot_iso3d!()","category":"page"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"Notes:","category":"page"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"the initial state of the quadrotor is expressed using a ComponentArray. This is an incredible library, and is the keytool to allow us to compose various equations together.\nnotice the |> when we constructed quad_p: we passed quad_p through initialize_quad_params.  This is necessary to initialize (and invert) some commonly used matrices, to avoid runtime cost\nRoboticSystems provides some convenient plotting tools.\nRS.plot_quad! plots a quadrotor\nRS.plot_triad plots the XYZ axes of a rotation matrix as red (x) green (y) and blue (z) lines. By default, we only use ENU frames, and quad_ic.R is a rotation matrix from the quadrotor-body fixed frame to to inertial frame.  \nRS.plot_iso3d! adjusts x,y,zlims to make the plot look like aspect_ratio=1 ","category":"page"},{"location":"quadrotor_geometric_control/#Define-the-closed-loop-function","page":"Quadrotor QuickStart","title":"Define the closed-loop function","text":"","category":"section"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"Next, we need to define the closed-loop dynamics, by using a feedback controller.  The controller used in this example is a geometric controller that uses differential flatness to track a path. ","category":"page"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"## Define a trajectory\n# a Lissajous trajectory\ntrajectory_params = ComponentArray(\n    A = [1, 1, 0.2],\n    ω = [5/8,4/8,6/8],\n    ϕ = [π/2, 0, 0],\n    off = [0,0,1.]\n)\n\n## Define the controller\n\ncontroller_params = ComponentArray(\n    kx = 1.0, \n    kv = 2.0,\n    kR = 0.35,\n    kΩ = 0.15, \n    m = 1.5,\n    J = quad_p.J, # this is cheating, but is convenient,\n    invG = quad_p.invG, # this is also cheating...\n    g = 9.81,\n    trajectory_params = trajectory_params\n)\n\nfunction controller(state, controller_params, t)\n\n    @unpack trajectory_params = controller_params\n\n    ## define the desired position, velocity, acceleration, jerk, snap at the current time\n    xd = RS.lissajous(t, trajectory_params)\n    vd = RS.lissajous(t, trajectory_params, 1)\n    ad = RS.lissajous(t, trajectory_params, 2)\n    jd = RS.lissajous(t, trajectory_params, 3)\n    sd = RS.lissajous(t, trajectory_params, 4)\n\n    ## define the desired yaw, yaw rate, and yaw acceleration at current time\n    ψ = 3.0*t/8\n    ψd = 3.0/8\n    ψdd = 0.0\n\n    ## convert into target quadrotor state,  using differential flatness\n    target = RS.flat_state_to_quad_state(xd, vd, ad, jd, sd, ψ, ψd, ψdd)\n\n    ## determine required thrust and moments, using geometric controller\n    fM = RS.geometric_controller(state, target..., controller_params)\n    # returns a vector [f, M[1], M[2], M[3]]\n\n    ## determine rotation speed of each motor\n    control = RS.fM_to_ω(fM, controller_params)\n    # returns the desired ω of each motor\n\nend\n\n\n## define the closed loop dynamics\nfunction cl_geometric_quad!(D, state, params, t)\n    \n    @unpack quad_params, controller_params = params\n\n    u = controller(state, controller_params, t)\n\n    # define the external wind speed (constant, for simplicity)\n    w_ext = [1.0, 0, 0.]\n\n    # determine the rate of change of state based on control input and other forces/torques\n    RS.quadrotor!(D, state, quad_params, u, t; wind_ext=w_ext)\n\nend","category":"page"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"You can imagine more complicated wind speeds can be modelled. Additional external forces or torques can be specified in RS.quadrotor!.  ","category":"page"},{"location":"quadrotor_geometric_control/#Perform-the-Simulation","page":"Quadrotor QuickStart","title":"Perform the Simulation","text":"","category":"section"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"We can use the excellent library DifferentialEquations.jl to achieve high performance and extensible ODE solving. RoboticSystems.jl doesnt require you to use DiffEq.jl, but it is written in a way that makes it easy to use DiffEq.jl. ","category":"page"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"using DifferentialEquations\n\ntspan = (0, 100.0)\n\n# combine all the parameters\nparams = ComponentArray(\n  quad_params = quad_p,\n  controller_params = controller_params\n)\n\n# define the ODE problem\nprob = ODEProblem(cl_geometric_quad!, quad_ic, tspan, params)\n\n## Solve the problem\nsol = solve(prob, Tsit5())\n","category":"page"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"All of the arguments (e.g. restol, abstol) that you normally pass into ODEProblem or solve can be used. Refer to the DifferentialEquations.jl documentation for more details. ","category":"page"},{"location":"quadrotor_geometric_control/#Plot/Animate-the-solution","page":"Quadrotor QuickStart","title":"Plot/Animate the solution","text":"","category":"section"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"plot()\nRS.plot_quad_traj!(sol, quad_p; tspan=tspan)\n\nplot!(xlims=(-1.5, 1.5), ylims=(-1.5, 1.5), zlims=(0, 1.5))\n\nRS.plot_iso3d!()\nRS.plot_project3d!()","category":"page"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"Next, we can use Plots.jl's animation tools to construct a simple animation. ","category":"page"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"anim = @animate for tm = range(sol.t[1], sol.t[end], 200)\n    \n    tspan=(sol.t[1], tm)\n    \n    plot()\n    RS.plot_quad_traj!(sol, quad_p; tspan=tspan)\n    \n    plot!(xlims=(-1.5, 1.5), ylims=(-1.5, 1.5), zlims=(0, 1.5))\n    \n    # rotate camera\n    plot!(camera = (360 * ((tm - sol.t[1]) / (sol.t[end] - sol.t[1])), 30))\n    plot!(axis = ([], false), xlabel=\"\", ylabel=\"\", zlabel=\"\")\n    RS.plot_iso3d!()\n    RS.plot_project3d!()\n\nend;\n\ngif(anim)","category":"page"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"To plot any additional plot, you can use sol as a function, for example:","category":"page"},{"location":"quadrotor_geometric_control/","page":"Quadrotor QuickStart","title":"Quadrotor QuickStart","text":"gr()\nplot(t-> RS.roll(sol(t).R) * 180 / π, tspan..., label=\"sim\")\nplot!(xlabel=\"time (s)\", ylabel=\"roll (degrees)\")","category":"page"}]
}
