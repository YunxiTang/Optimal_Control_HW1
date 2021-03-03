
# Test a random QP
using Random
Random.seed!(2)
n,m,p = 10,0,15 
qp = HW1.QPData(n,m,p)
P = rand(n,n)
qp.P .= P'P   # make it P.S.D
qp.q .= randn(n)
qp.A .= randn(m,n)
qp.b .= randn(m)
qp.C .= randn(p,n)
qp.d .= randn(p)

# Initial guess
x = randn(n)

# Solve
xstar, λstar, μstar = HW1.solve_qp(qp, x)

# Check optimality conditions
@test norm(HW1.primal_residual(qp, xstar, λstar, μstar)) < 1e-3
@test norm(HW1.dual_residual(qp, xstar, λstar, μstar)) < 1e-6
@test norm(HW1.complimentarity(qp, xstar, λstar, μstar)) < 1e-3
@test all(μstar .>= 0)

# Compare with OSQP
using OSQP, SparseArrays
model = OSQP.Model()
OSQP.setup!(model, P=sparse(qp.P), q=qp.q, A=sparse([qp.A; qp.C]), l=[qp.b; fill(-Inf,p)], u=[qp.b; qp.d],
    eps_abs=1e-6, eps_rel=1e-6)
res = OSQP.solve!(model)
@test norm(res.x - xstar) < 1e-3
@test norm(res.y - [λstar; μstar]) < 1e-3


# Build QP
q = [1.2,3.5]
v = [-1.5,2.6]
qp = HW1.build_qp(q,v)
h = 0.01
g = [0,9.81]
@test qp.P ≈ I(2)
@test qp.q ≈ qp.P*(h*g - v)
@test isempty(qp.A)
@test isempty(qp.b)
@test size(qp.C) == (1,2)
@test size(qp.d) == (1,)
@test qp.C ≈ [0 -h]
@test qp.d ≈ [q[2]]

# Simulation
qs,vs = HW1.simulate_brick()
qs[1] ≈ [0,1]
vs[1] ≈ [1,0]
@test length(qs) == 301
@test qs[end] ≈ [3,0] atol=1e-6
@test vs[end] ≈ [1,0] atol=1e-10
height = [q[2] for q in qs]
@test minimum(height) > -1e-4