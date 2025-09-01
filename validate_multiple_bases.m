load("usd_robot_params.mat");
% drive_gains = [14.87; 13.26; 11.13; 10.62; 11.03; 11.47]; 
pi_b = baseQR.permutationMatrix(:, 1:baseQR.numberOfBaseParameters)' * sol.pi_s;
isaac_b = baseQR.permutationMatrix(:, 1:baseQR.numberOfBaseParameters)' * usd_params.pi_standard';

param_sets = create_param_sets( ...
    isaac_b, sol.pi_fr, [30, 21, 18, 9, 16, 1], 'isaac dynamic', ...
    sol.pi_b, sol.pi_fr, [10, 10.7, 8.45, 9, 9.48, 0.07], 'base', ...
    pi_b, sol.pi_fr, [30,22,16,40,16,0.07], 'dynamic');


n_links = 6;
rre = plot_multi_params(path_to_val_data, idxs, baseQR, drive_gains, n_links, param_sets);