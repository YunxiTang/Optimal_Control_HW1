

@testset "Question 2" begin
model = HW1.UnitreeA1()
@testset "Part a" begin
n = 30
m = 12
x = rand(n)
u = rand(m)
λ = rand(n)
A = rand(n,n)
B = rand(n,m)
r = HW1.kkt_conditions(model, x, u, λ, A, B)
@test length(r) == 2n+m 

H = HW1.kkt_jacobian(model, x, u, λ, A, B)
@test size(H) == (2n+m, 2n+m)
@test isdiag(H[1:n,1:n])
@test isdiag(H[n+m+1:end, n+m+1:end])
@test issymmetric(H)

H2 = HW1.kkt_jacobian(model, x, u, λ, A, B, 1e-2)
@test isdiag(H2-H)
end

@testset "Part b" begin
xstar,ustar,λstar, hist = HW1.newton_solve(model, HW1.initial_state(model), verbose=false)
@test norm(HW1.dynamics(model, xstar, ustar)) < 1e-6
@test length(hist) < 100
end

@testset "Part c" begin
@test HW1.final_residual < 1e-6
@test any(real.(HW1.eigs) .> 0)
end
end