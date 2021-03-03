using RigidBodyDynamics
using MeshCat
using MeshCatMechanisms
using Random
using StaticArrays
using Rotations
using LinearAlgebra
using ForwardDiff

const URDFPATH = joinpath(@__DIR__, "..","a1","urdf","a1.urdf")
const STATEMAP = (
    foot_joint_x = (1,1),
    foot_joint_y = (2,2),
    foot_joint_z = (3,3),
    FR_hip_joint = (4,7),
    FL_hip_joint = (5,8),
    RR_hip_joint = (6,6),
    RL_hip_joint = (7,9),
    FR_thigh_joint = (8,10),
    FL_thigh_joint = (9,11),
    RR_thigh_joint = (10,5),
    RL_thigh_joint = (11,12),
    FR_calf_joint = (12,13),
    FL_calf_joint = (13,14),
    RR_calf_joint = (14,4),
    RL_calf_joint = (15,15)
)
function state2mech(x)
    SA[
        x[STATEMAP[:foot_joint_x][1]],
        x[STATEMAP[:foot_joint_y][1]],
        x[STATEMAP[:foot_joint_y][1]],
        x[STATEMAP[:RR_calf_joint][1]],
        x[STATEMAP[:RR_thigh_joint][1]],
        x[STATEMAP[:RR_hip_joint][1]],
        x[STATEMAP[:FR_hip_joint][1]],
        x[STATEMAP[:FL_hip_joint][1]],
        x[STATEMAP[:RL_hip_joint][1]],
        x[STATEMAP[:FR_thigh_joint][1]],
        x[STATEMAP[:FL_thigh_joint][1]],
        x[STATEMAP[:RL_thigh_joint][1]],
        x[STATEMAP[:FR_calf_joint][1]],
        x[STATEMAP[:FL_calf_joint][1]],
        x[STATEMAP[:RL_calf_joint][1]],
    ]
end


function mech2state(q)
    SA[
        q[STATEMAP[:foot_joint_x][2]],
        q[STATEMAP[:foot_joint_y][2]],
        q[STATEMAP[:foot_joint_z][2]],
        q[STATEMAP[:FR_hip_joint][2]],
        q[STATEMAP[:FL_hip_joint][2]],
        q[STATEMAP[:RR_hip_joint][2]],
        q[STATEMAP[:RL_hip_joint][2]],
        q[STATEMAP[:FR_thigh_joint][2]],
        q[STATEMAP[:FL_thigh_joint][2]],
        q[STATEMAP[:RR_thigh_joint][2]],
        q[STATEMAP[:RL_thigh_joint][2]],
        q[STATEMAP[:FR_calf_joint][2]],
        q[STATEMAP[:FL_calf_joint][2]],
        q[STATEMAP[:RR_calf_joint][2]],
        q[STATEMAP[:RL_calf_joint][2]],
    ]
end

function controls2torques(u)
    τ = [SA[0,0,0]; u]
    state2mech(τ)
end

function torques2controls(τ)
    u = mech2state(τ) 
    return u[SVector{12}(4:15)]
end

function mechstate2fullstate(state::MechanismState)
    return [mech2state(configuration(state)); mech2state(velocity(state))]
end

function attach_foot!(mech::Mechanism{T}, foot="RR"; revolute::Bool=true) where T
    # Get the relevant bodies from the model
    foot = findbody(mech, foot * "_foot")
    trunk = findbody(mech, "trunk")
    world = findbody(mech, "world")

    # Get the location of the foot
    state = MechanismState(mech)
    trunk_to_foot = translation(relative_transform(state, 
        default_frame(trunk), default_frame(foot)))
    foot_location = SA[trunk_to_foot[1], trunk_to_foot[2], 0]  # set the z height to 0

    # Build the new foot joint
    if !revolute 
        foot_joint = Joint("foot_joint", QuaternionSpherical{T}())
        world_to_joint = Transform3D(
            frame_before(foot_joint),
            default_frame(world),
            -foot_location        
        )

        # Attach to model
        attach!(mech,
            world,
            foot,
            foot_joint,
            joint_pose = world_to_joint,
        )
        remove_joint!(mech, findjoint(mech, "base_to_world"))
    else
        # Create dummy bodies 
        dummy1 = RigidBody{T}("dummy1")
        dummy2 = RigidBody{T}("dummy2")
        for body ∈ (dummy1, dummy2)
            inertia = SpatialInertia(default_frame(body),
                moment = I(3)*1e-3,
                com    = SA[0,0,0],
                mass   = 1e-3
            )
            spatial_inertia!(body, inertia)
        end

        # X-Joint
        foot_joint_x = Joint("foot_joint_x", Revolute{T}(SA[1,0,0]))
        world_to_joint = Transform3D(
            frame_before(foot_joint_x),
            default_frame(world),
            -foot_location        
        )
        attach!(mech,
            world,
            dummy1,
            foot_joint_x,
            joint_pose = world_to_joint
        )

        # Y-Joint
        foot_joint_y = Joint("foot_joint_y", Revolute{T}(SA[0,1,0]))
        dummy_to_dummy = Transform3D(
            frame_before(foot_joint_y),
            default_frame(dummy1),
            SA[0,0,0]
        )
        attach!(mech,
            dummy1,
            dummy2,
            foot_joint_y,
            joint_pose = dummy_to_dummy 
        )

        # Z-Joint
        foot_joint_z = Joint("foot_joint_z", Revolute{T}(SA[0,0,1]))
        joint_to_foot = Transform3D(
            frame_before(foot_joint_z),
            default_frame(dummy2),
            SA[0,0,0]
        )
        attach!(mech,
            dummy2,
            foot,
            foot_joint_z,
            joint_pose = joint_to_foot
        )
        remove_joint!(mech, findjoint(mech, "base_to_world"))
    end
end

function build_quadruped()
    a1 = parse_urdf(URDFPATH, floating=true, remove_fixed_tree_joints=false) 
    attach_foot!(a1)
    # build_rear_foot_constraints_revolute!(a1)
    return a1
end

struct UnitreeA1{C}
    mech::Mechanism{Float64}
    statecache::C
    dyncache::DynamicsResultCache{Float64}
    xdot::Vector{Float64}
    function UnitreeA1(mech::Mechanism)
        N = num_positions(mech) + num_velocities(mech)
        statecache = StateCache(mech)
        rescache = DynamicsResultCache(mech)
        xdot = zeros(N)
        new{typeof(statecache)}(mech, statecache, rescache, xdot)
    end
end
function UnitreeA1()
    UnitreeA1(build_quadruped())
end

state_dim(model::UnitreeA1) = 30
control_dim(model::UnitreeA1) = 12 
function get_partition(model::UnitreeA1)
    n,m = state_dim(model), control_dim(model)
    return 1:n, n .+ (1:m), n+m .+ (1:n)
end

function dynamics(model::UnitreeA1, x::AbstractVector{T1}, u::AbstractVector{T2}) where {T1,T2} 
    T = promote_type(T1,T2)
    state = model.statecache[T]
    res = model.dyncache[T]

    # Convert from state ordering to the ordering of the mechanism
    # q = state2mech(x[SVector{15}(1:15)]) 
    # v = state2mech(x[SVector{15}(16:30)])
    copyto!(state, x)
    # τ = controls2torques(u)
    τ = [zeros(3); u]
    # set_configuration!(state, q)
    # set_velocity!(state, v)
    dynamics!(res, state, τ)
    # q̇ = mech2state(res.q̇)
    # v̇ = mech2state(res.v̇)
    q̇ = res.q̇
    v̇ = res.v̇
    return [q̇; v̇]
end

function jacobian(model::UnitreeA1, x, u)
    ix = SVector{30}(1:30)
    iu = SVector{12}(31:42)
    faug(z) = dynamics(model, z[ix], z[iu])
    z = [x; u]
    ForwardDiff.jacobian(faug, z)
end

function rk4(model::UnitreeA1, x, u, dt)
	k1 = dynamics(model, x,        u)*dt
	k2 = dynamics(model, x + k1/2, u)*dt
	k3 = dynamics(model, x + k2/2, u)*dt
	k4 = dynamics(model, x + k3,   u)*dt
	x + (k1 + 2k2 + 2k3 + k4)/6
end

# Set initial guess
function initial_state(model::UnitreeA1)
    state = model.statecache[Float64]
    a1 = model.mech
    zero!(state)
    leg = ("FR","FL","RR","RL")
    for i = 1:4
        s = isodd(i) ? 1 : -1
        f = i < 3 ? 1 : -1
        set_configuration!(state, findjoint(a1, leg[i] * "_hip_joint"), deg2rad(-20s))
        set_configuration!(state, findjoint(a1, leg[i] * "_thigh_joint"), deg2rad(-30f))
        set_configuration!(state, findjoint(a1, leg[i] * "_calf_joint"), deg2rad(10f))
    end
    set_configuration!(state, findjoint(a1, "foot_joint_x"), deg2rad(00))
    set_configuration!(state, findjoint(a1, "foot_joint_y"), deg2rad(-00))

    return [configuration(state); velocity(state)]
end

function initialize_visualizer(a1::UnitreeA1)
    vis = Visualizer()
    delete!(vis)
    cd(joinpath(@__DIR__,"..","a1","urdf"))
    mvis = MechanismVisualizer(a1.mech, URDFVisuals(URDFPATH), vis)
    cd(@__DIR__)
    return mvis
end

# times = range(0,1, step=0.01)
# X = [zero(x) for t in times]
# X[1] .= x
# for i = 1:length(times)-1
#     X[i+1] = rk4(model, X[i], u, times[i+1]-times[i])
# end
# qs = [state2mech(x[1:15]) for x in X]
# animate(mvis, Vector(times), Vector.(qs))