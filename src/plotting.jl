
@userplot Plot_Triad
@recipe function f(robot::Plot_Triad; L = 0.1)
    state = robot.args[1]
    @unpack x, R = state

    pts = zeros(3, 3 * 3)
    for i = 1:3
        pts[:, (i-1)*3+1] = x
        pts[:, (i-1)*3+2] = x + L * R[:, i]
        pts[:, (i-1)*3+3] .= NaN
    end

    linecolor --> [:red, :black, :black, :green, :black, :black, :blue, :black]
    label --> false
    xlabel --> "x [m]"
    ylabel --> "y [m]"
    zlabel --> "z [m]"

    pts[1, :], pts[2, :], pts[3, :]
end



function plot_iso3d!()
    x12, y12, z12 = xlims(), ylims(), zlims()
    d = maximum([diff([x12...]), diff([y12...]), diff([z12...])])[1] / 2
    xm, ym, zm = mean(x12), mean(y12), mean(z12)

    plot!(xlims = (xm - d, xm + d), ylims = (ym - d, ym + d), zlims = (zm - d, zm + d))
end

function plot_project3d!(; side = [1, 2, 1], color = :lightgray)
    pl = plot!()
    ss = copy(pl.series_list)
    xl = xlims()[side[1]]
    yl = ylims()[side[2]]
    zl = zlims()[side[3]]
    for s in ss
        N = length(s[:x])
        plot!(xl * ones(N), s[:y], s[:z], color = color, legend = false)
        plot!(s[:x], yl * ones(N), s[:z], color = color, legend = false)
        plot!(s[:x], s[:y], zl * ones(N), color = color, legend = false)
    end
    plot!()
end
