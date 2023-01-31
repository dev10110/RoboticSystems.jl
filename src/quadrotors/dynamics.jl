


function motor_thrust(ω, k_f)

    return k_f * signedsq(ω)
end

function hover_ω(params)
    @unpack k_f, mass, g = params
    return SVector{4}(sqrt(mass * g / (4 * k_f)) for i = 1:4)
end

function f_grav(state, params)
    @unpack mass, g = params
    return SA[0.0, 0, -mass*g]
end

function f_motors(state, params)
    @unpack ω = state
    @unpack k_f = params
    # in body frame
    return (SA[0.0, 0, motor_thrust(ω[i], k_f)] for i = 1:4) |> collect
end

function f_thrust(state, params)
    @unpack R = state
    # R is body-> world
    return SMatrix{3,3}(R) * sum(f_motors(state, params))
end

function f_drag(state, params, wind_ext)
    @unpack v = state
    @unpack k_drag_f = params

    δv = SVector{3}(v - wind_ext)

    return -k_drag_f * norm(δv) * δv
end

function μ_thrust(Dω, state, params)
    @unpack Ω, ω = state
    @unpack J_m, k_μ, motor_pos, motor_dir = params

    # contribution from thrust of each motor
    f_motors_ = f_motors(state, params)
    μ_1 = (sum(cross(motor_pos[:, i], f_motors_[i]) for i = 1:4))

    # contribution from acceleration of each motor
    μ_2 = SA[0, 0, sum(-motor_dir[i] * (k_μ * signedsq(ω[i]) + J_m * (Dω)[i]) for i = 1:4)]
    μ_thrust = μ_1 + μ_2

    return μ_thrust
end

function μ_drag(state, params)
    @unpack Ω = state
    @unpack k_drag_μ = params
    return -k_drag_μ * norm(Ω) * SVector{3}(Ω)
end

function angular_momentum(state, params)
    @unpack Ω, ω = state
    @unpack J, motor_dir, J_m = params

    H = SVector{3}(J * Ω) - SA[0, 0, sum(motor_dir[i] * J_m * ω[i] for i = 1:4)]

    return H
end

function quadrotor!(
    D,
    state,
    params,
    ω_des,
    t;
    wind_ext = SA[0, 0, 0.0],
    f_ext = SA[0, 0, 0.0],
    μ_ext = SA[0, 0, 0.0],
)

    @unpack x, v, R, Ω, ω = state
    @unpack mass,
    J,
    τ_m,
    J_m,
    g,
    k_f,
    k_μ,
    k_drag_f,
    k_drag_μ,
    motor_pos,
    motor_dir,
    ω_max = params

    ## Motor Dynamics
    D.ω = (1 / τ_m) .* (clamp.(ω_des, 0, ω_max) - ω)

    ## Position Dynamics
    f_total =
        f_grav(state, params) +
        f_thrust(state, params) +
        f_drag(state, params, wind_ext) +
        f_ext
    D.x = v
    D.v = f_total / mass

    ## Rotational Dynamics
    μ_total = μ_thrust(D.ω, state, params) + μ_drag(state, params) + μ_ext
    H = angular_momentum(state, params)

    D.R = R * hat(Ω)
    D.Ω = J \ (μ_total - cross(Ω, H))

    return

end
