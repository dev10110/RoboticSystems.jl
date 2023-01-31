
function lissajous(t, params, D::Integer = 0)

    @unpack A, ω, ϕ, off = params
    return lissajous(t, A, ω, ϕ, off, D)
end


function lissajous(t, A, ω, ϕ, off, D = 0)

    N = length(A)

    if D == 0
        return SVector{N}(off[i] + A[i] * sin(ω[i] * t + ϕ[i]) for i = 1:N)
    else
        return SVector{N}(A[i] * ω[i]^D * sin(ω[i] * t + π * D / 2 + ϕ[i]) for i = 1:N)
    end
end



# lissajous_parameters = ComponentArray(
#     A = [1, 1, 0.2],
#     ω = [5/8,4/8,6/8],
#     ϕ = [π/2, 0, 0],
#     off = [0,0,1.]
# )
# 
# 
