function test_base_params(DOF)
% ---------------------------------------------------------------------
% Test if base parameters are found correctly by comparing
% torque prediction of the model with standard parameters with 
% the torque prediction of the model with base parameters
% ----------------------------------------------------------------------

path_to_urdf = 'ur10e.urdf';
ur10 = parse_urdf(path_to_urdf, DOF);


% Perform QR decompostions
include_motor_dynamics = 1;
[~, baseQR] = base_params_qr(include_motor_dynamics, DOF);

bb = baseQR.numberOfBaseParameters;
E = baseQR.permutationMatrix;
beta = baseQR.beta;
includeMotorDynamics = baseQR.motorDynamicsIncluded;

if includeMotorDynamics
    no_link_params = 11;
    ur10.pi(end+1,:) = rand(1,DOF);
else
    no_link_params = 10;
end
ur10.pi = reshape(ur10.pi,[no_link_params*DOF, 1]);

% Position, velocity and acceleration limits
q_min = -pi*ones(DOF,1);
q_max = pi*ones(DOF,1);
qd_max = 3*pi*ones(DOF,1);
q2d_max = 6*pi*ones(DOF,1);

% On random positions, velocities, aceeleations
for i = 1:100
    q_rnd = q_min + (q_max - q_min).*rand(DOF,1);
    qd_rnd = -qd_max + 2*qd_max.*rand(DOF,1);
    q2d_rnd = -q2d_max + 2*q2d_max.*rand(DOF,1);
    
    if includeMotorDynamics
        Yi = regressorWithMotorDynamics(q_rnd,qd_rnd,q2d_rnd);
    else
        Yi = standard_regressor_UR10E(q_rnd,qd_rnd,q2d_rnd);
    end
    tau_full = Yi*ur10.pi;
    
    pi_lgr_base = [eye(bb) beta]*E'*ur10.pi;
    Y_base = Yi*E(:,1:bb);
    tau_base = Y_base*pi_lgr_base;
    assert(norm(tau_full - tau_base) < 1e-6);
end
fprintf("Rigid Body Base Dynamics Test - OK!\n");