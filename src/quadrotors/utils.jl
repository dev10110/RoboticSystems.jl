
QuadStateAxes = Axis(
    x = 1:3,
    v = 4:6,
    R = ViewAxis(7:15, ShapedAxis((3, 3), NamedTuple())),
    Ω = 16:18,
    ω = 19:22,
)
QuadTargetStateAxes =
    Axis(xd = 1:3, vd = 4:6, ad = 7:9, b1d = 10:12, Ωd = 13:15, αd = 16:18)
QuadFlatStateAxes =
    Axis(x = 1:3, v = 4:6, a = 7:9, j = 10:12, s = 13:15, ψ = 16, ψd = 17, ψdd = 18)

function initialize_quad_params(quad_p)

    @unpack J, k_f, k_μ, motor_pos, motor_dir = quad_p

    invJ = inv(J)

    G = zeros(4, 4)

    Gf = k_f * ones(1, 4)

    GM = zeros(3, 4)
    for i = 1:4
        G[1, i] = k_f
        G[2:4, i] = cross(motor_pos[:, i], [0, 0, k_f])
        G[4, i] = -motor_dir[i] * k_μ
    end

    invG = inv(G)

    return ComponentArray(quad_p; invJ = invJ, invG = invG)
end

quad_LX = 0.08
quad_LY = 0.08
const quadrotor_parameters =
    ComponentArray(
        mass = 1.0,            # mass of quadrotor [kg]
        J = diagm([0.0049, 0.0049, 0.0069]) |> collect,                # moment of inertia of quadrotor in body axes [kg/m^2]
        τ_m = 0.02,            # time constant for quadrotor motors [s]
        J_m = 6.62e-6,         # moment of inertia of each motor+propeller [kg/m^2]
        g = 9.81,              # acceleration due to gravity [m/s^2]
        k_f = 1.91e-6,         # thrust coefficient, [N / (rad/s)^2]
        k_μ = 2.7e-7,          # torque coefficient, [Nm / (rad/s)^2]
        k_drag_f = 0.1,        # linear drag coefficient, [N / (m/s)^2]
        k_drag_μ = 0.003 * I(3), # angular drag coefficient, [Nm / (rad/s)^2]
        ω_max = 5000.0,          # maximum rotation rate of motors [rad/s]
        motor_pos = hcat(
            [
                [quad_LX, -quad_LY, 0.0],
                [-quad_LX, quad_LY, 0.0],
                [quad_LX, quad_LY, 0.0],
                [-quad_LX, -quad_LY, 0.0],
            ]...,
        ),                        # motor positions in body-fixed frame (i-th column contains i-th motor pos)
        motor_dir = [1, 1, -1, -1], # motor directions in body-fixed frame
    ) |> initialize_quad_params
