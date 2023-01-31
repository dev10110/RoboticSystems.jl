
@userplot Plot_Quad
@recipe function f(quad::Plot_Quad)
    state = quad.args[1]
    params = quad.args[2]
    @unpack x, R = state
    @unpack motor_pos = params

    pts = zeros(3, 3 * 4)
    for i = 1:4
        pts[:, (i-1)*3+1] = x
        pts[:, (i-1)*3+2] = x + R * motor_pos[:, i]
        pts[:, (i-1)*3+3] .= NaN
    end

    linecolor -->
    [:red, :black, :black, :black, :black, :black, :red, :black, :black, :black, :black]
    label --> false
    xlabel --> "x [m]"
    ylabel --> "y [m]"
    zlabel --> "z [m]"
    linewidth --> 2

    pts[1, :], pts[2, :], pts[3, :]
end



function plot_quad_traj!(
    sol,
    quad_params;
    tspan = (sol.t[1], sol.t[end]),
    N = 1,
    prefix = "",
    kwargs...,
)

    t0, t1 = tspan

    @assert N >= 1
    ts = N == 1 ? [t1] : range(t0, t1, length = N)

    for t in ts
        if prefix == ""
            plot_quad!(sol(t), quad_params)
        else
            plot_quad!(sol(t)[prefix], quad_params)
        end
    end

    if prefix == ""
        trajx = t -> sol(t).x[1]
        trajy = t -> sol(t).x[2]
        trajz = t -> sol(t).x[3]
        label = ""
    else
        trajx = t -> sol(t)[prefix].x[1]
        trajy = t -> sol(t)[prefix].x[2]
        trajz = t -> sol(t)[prefix].x[3]
        label = prefix
    end

    plot!(trajx, trajy, trajz, t0:0.01:t1; label = label, kwargs...)

end
