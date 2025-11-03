load("usd_robot_params.mat");
pi_b = baseQR.permutationMatrix(:, 1:baseQR.numberOfBaseParameters)' * sol.pi_s;
isaac_b = baseQR.permutationMatrix(:, 1:baseQR.numberOfBaseParameters)' * usd_params.pi_standard';

% scales=[25, 150, 50, 3, 0.2, 0.003];
% param_sets = create_param_sets( ...
%     isaac_b, sol.pi_fr, [4,2,2.2,1,0.8,0.005], 'isaac dynamic', ...
%     sol.pi_b, sol.pi_fr, drive_gains'*diag(scales), 'base', ...
%     pi_b, sol.pi_fr, drive_gains'*diag(scales), 'dynamic');
% 
% rre = plot_multi_params(path_to_val_data, idxs, baseQR, drive_gains'*diag(scales), param_sets);


scales=[25, 150, 50, 3, 0.2, 0.003];
param_sets = create_param_sets_frictionless( ...
    isaac_b, [1,1,1,1,1,1], 'isaac dynamic', ...
    sol.pi_b, drive_gains'*diag(scales), 'base', ...
    pi_b, drive_gains'*diag(scales), 'dynamic');

rre = plot_multi_params_frictionless(path_to_val_data, idxs, baseQR, drive_gains'*diag(scales), param_sets);