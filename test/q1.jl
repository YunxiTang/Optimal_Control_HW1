

@testset "Q1" begin
q1 = load(joinpath(@__DIR__, "q1.jld2"))
x1 = [1,1.] 
x2 = [2,2.] 
h = 0.1
@testset "Part a" begin
out = HW1.implicit_midpoint_pendulum(x2,x1,h)
@test out ≈ q1["out"] 
@test out !== x2
@test out !== x1
@test x2 ≈ [2,2]
end

@testset "Part b" begin
res = HW1.implicit_midpoint_solve!(x2, x1, h)
@test res[end] < 1e-6
@test norm(HW1.implicit_midpoint_pendulum(x2,x1,h) - x2) < 1e-6
@test mean(HW1.convergence_rate(res)) > 1.5
@test x2 ≈ q1["x2"] 
end

@testset "Part c" begin
E = HW1.energy_implicit
@test length(E) == 36001
@test (mean(E) - 2.874) < 1e-2 
@test std(E) < 1e-2
@test abs(E[end] - E[1]) < 1e-2
end

@testset "Part d" begin
model = RobotZoo.Pendulum(1., 0., 0., 1., 0., 9.81)
for i = 1:5
    x = @SVector rand(2) 
    @test discrete_dynamics(RK4, model, x, SA[0.], 0., h) ≈ HW1.rk4(x, h) atol=1e-6
end

E2 = HW1.energy_rk4
@test E2[end] - E2[1] < 0
@test E2[end] - E2[1] < -0.9
@test std(diff(E2)) < 1e-2
end

@testset "Part e" begin
Amid = q1["Amid"] 
@test HW1.Amid  ≈ Amid atol=1e-6 
@test abs.(eigvals(HW1.Amid)) ≈ ones(2) atol=1e-8
@test std(HW1.eigs_implicit) < 1e-10
@test mean(HW1.eigs_implicit) ≈ 1 atol=1e-10
end
end