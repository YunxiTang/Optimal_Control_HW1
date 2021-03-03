function convergence_rate(v::AbstractVector{T}) where T
    n = length(v)
    r = zeros(n-1)
    for i = 1:n-1
        if v[i] > one(T)
            @inbounds a,b = v[i], v[i+1]
        else
            @inbounds a,b = v[i+1], v[i]
        end
        @inbounds r[i] = abs(log10(a) / log10(b))
    end
    return r
end