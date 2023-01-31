
function hat(v)
    return [
        [0;; -v[3];; v[2]]
        [v[3];; 0;; -v[1]]
        [-v[2];; v[1];; 0]
    ]
end

function hat(v::SV3{F}) where {F}
    return @SMatrix [
        [zero(F);; -v[3];; v[2]]
        [v[3];; zero(F);; -v[1]]
        [-v[2];; v[1];; zero(F)]
    ]
end

function vee(M)
    return [
        (M[3, 2] - M[2, 3]) / 2
        (M[1, 3] - M[3, 1]) / 2
        (M[2, 1] - M[1, 2]) / 2
    ]
end

function vee(M::SM3)
    return @SVector [
        (M[3, 2] - M[2, 3]) / 2
        (M[1, 3] - M[3, 1]) / 2
        (M[2, 1] - M[1, 2]) / 2
    ]
end


signedsq(a) = a * abs(a)
signedsqrt(a) = sign(a) * sqrt(abs(a))
mean(x) = sum(x) / length(x)



roll(rotMat) = RotXYZ(rotMat).theta1
pitch(rotMat) = RotXYZ(rotMat).theta2
yaw(rotMat) = RotXYZ(rotMat).theta3
