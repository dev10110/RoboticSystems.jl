
function fM_to_ω(fM, quad_params)
    @unpack invG = quad_params

    ωsq = SMatrix{4,4}(invG) * SVector{4}(fM)

    return SVector{4}(signedsqrt(w) for w in ωsq)
end



"""
Based on "Accurate Tracking of Aggressive Quadrotor Trajectories using Incremental Nonlinear Dynamic Inversion and Differential Flatness" by Ezra Tal, Sertac Karaman
"""
function flat_state_to_quad_state(flat_states::ComponentVector{F}, g::F = F(9.81)) where {F}

    @unpack x, v, a, j, s, ψ, ψd, ψdd = flat_states

    out = flat_state_to_quad_state(x, v, a, j, s, ψ, ψd, ψdd, g)

    return ComponentArray(vcat(out...), QuadTargetStateAxes)

end

# """
# Based on "Accurate Tracking of Aggressive Quadrotor Trajectories using Incremental Nonlinear Dynamic Inversion and Differential Flatness" by Ezra Tal, Sertac Karaman
# """
function flat_state_to_quad_state(
    x::VF,
    v::VF,
    a::VF,
    j::VF,
    s::VF,
    ψ::F,
    ψd::F,
    ψdd::F,
    g = 9.81,
) where {F,VF<:AbstractArray{F}}

    # desired rotation matrix
    force = SA[a[1], a[2], a[3]+g]
    zb = normalize(force)
    xc = SA[cos(ψ), sin(ψ), 0]
    yb = cross(zb, xc) |> normalize
    xb = cross(yb, zb) |> normalize

    R = SMatrix{3,3}([xb yb zb])
    b1d = R * SA[1, 0, 0]

    # construct τ
    τ = norm(force)

    # construct S matrix
    bx1 = R[1, 1]
    bx2 = R[2, 1]
    bx3 = R[3, 1]
    by1 = R[1, 2]
    by2 = R[2, 2]
    by3 = R[3, 2]
    bz1 = R[1, 3]
    bz2 = R[2, 3]
    bz3 = R[3, 3]

    # 1x3 matrix
    S = SA[
        0;;
        (bx2 * bz1 - bx1 * bz2) / (bx1^2 + bx2^2);;
        (-bx2 * by1 + bx1 * by2) / (bx1^2 + bx2^2)
    ]

    # solve for Ω, τdot
    iz = SA[0, 0, 1]
    hatizT = hat(iz)'

    bz = R[:, 3]

    M = zero(MMatrix{4,4,F,16})
    M[1:3, 1:3] = τ * R * hatizT
    M[1:3, 4] = bz
    M[4, 1:3] = S
    invM = inv(SMatrix{4,4,F,16}(M))

    Ωτd = invM * [SVector{3}(j); ψd]

    Ω = SV3{F}(Ωτd[1], Ωτd[2], Ωτd[3])
    τd = Ωτd[4]

    # construct Sdot matrix
    # expression derived using mathematica
    w1, w2, w3 = Ω[1], Ω[2], Ω[3]

    Sd = SA[
        0;;
        (bx1 * Ω[1]) / (bx1^2 + bx2^2) +
        (bx2 * Ω[2]) / (bx1^2 + bx2^2) +
        ((bx1^2 * bz1 - bx2^2 * bz1 + 2 * bx1 * bx2 * bz2) * Ω[3]) / (bx1^2 + bx2^2)^2;;
        ((bx1^2 * bx2 + bx2^3 - bx1^2 * by1 + bx2^2 * by1 - 2 * bx1 * bx2 * by2) * Ω[3]) / (bx1^2 + bx2^2)^2
    ]

    # solve for α, τdd
    B1 = R * (2 * τd * hatizT + τ * hat(Ω) * hatizT) * Ω
    B2 = (Sd * Ω)
    B = [B1; B2]
    ατdd = invM * [SVector{3}(s - B1); (ψdd - B2[1])]

    α = SV3{F}(ατdd[1], ατdd[2], ατdd[3])
    #     τdd = ατdd[4]

    return x, v, a, b1d, Ω, α

end

# """
# Based on "Trajectory Generation and Control for Quadrotors" by Daniel Mellinger 2011
# """
# function flat_state_to_quad_state_Mellinger(flat_states, g = 9.81)

#     @unpack x, v, a, j, s, ψ, ψd, ψdd = flat_states

#     # desired rotation matrix
#     # start at eq. 2.19
#     zb = [a[1], a[2], a[3] + g] |> normalize
#     xc = [cos(ψ), sin(ψ), 0]
#     yb = cross(zb, xc) |> normalize
#     xb = cross(yb, zb) |> normalize

#     R = [xb yb zb]
#     b1d = R * [1,0,0]

#     # desired angular velocity
#     τ = norm([a[1], a[2], a[3] + g])
#     τd = dot(a, zb) - dot([0, 0,-g], zb) # τ = u_1 / mass

#     hω = (1 / τ) * (j - dot(zb, j) * j)

#     p = -dot( hω, yb)
#     q = dot(hω, zb)

#     # this equation makes little sense to me
#     # see eq 2.22
#     zw = [0,0,1]
#     Rweird = [xc yb zw]
#     M = Rweird \ R
#     r = (ψd - M[3,1]*p - M[3,2]*q) / M[3,3]

#     Ωd = [p,q,r]

#     ## desired angular acceleration
#     τdd = dot(zb, s) - dot(zb, cross(Ωd, cross(Ωd, τ*zb)))
#     hα = (1/τ) * (j - τdd*zb - 2*cross(Ωd, τd * zb) - cross(Ωd, cross(Ωd, τ*zb)) )

#     pd = -dot(hα, yb)
#     qd = dot(hα, xb)

#     ϕd, θd, ψd = M*Ωd

#     ωcw = [0,0,ψd]

#     B = -  Rweird \ (cross(ωcw, ϕd*xc) + cross(Ωd, θd*yb))

#     rd = (1/M[3,3]) *(ψdd - B[3] - (M[3,1]*pd + M[3,2]*qd))

#     αd = [pd, qd, rd]

#     target_state = ComponentArray(
#         xd = x,
#         vd = v,
#         ad = a,
#         b1d = b1d,
#         Ωd = Ωd,
#         αd = αd
#     )

#     return target_state
# 
# end



function geometric_controller(state, xd, vd, ad, b1d, Ωd, αd, controller_params)

    @unpack x, v, R, Ω = state
    @unpack kx, kv, kR, kΩ, m, J, g = controller_params

    e3 = [0, 0, 1]

    ex = x - xd
    ev = v - vd

    # construct desired rotation matrix
    b3d = (-kx * ex - kv * ev + m * g * e3 + m * ad) |> normalize
    b2d = cross(b3d, normalize(b1d)) |> normalize
    b1d_n = cross(b2d, b3d) |> normalize

    Rd = [b1d_n b2d b3d]

    eR = 0.5 * vee(Rd' * R - R' * Rd)
    eΩ = Ω - R' * Rd * Ωd

    f = dot(-kx * ex - kv * ev + m * g * e3 + m * ad, R * e3)
    M = -kR * eR - kΩ * eΩ + cross(Ω, (J * Ω)) - J * (hat(Ω) * R' * Rd * Ωd - R' * Rd * αd)

    return [f, M...]

end
