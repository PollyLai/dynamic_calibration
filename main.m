clear all; close all; clc;

% Define path to a urdf file
path_to_urdf = 'ur10e.urdf';
n_links = 6;
% path_to_urdf = 'fixed_bipedal.urdf';
% n_links = 10;    
% Generate functions for dynamics based on Lagrange method
% Note that it might take some timesqlpdemo
% generate_rb_dynamics(path_to_urdf, n_links);  \\ some errors
generate_friction_eq(n_links);


% Generate regressors for inverse dynamics of the robot, friction and load
% Note that it might take some time
generate_rb_regressor(path_to_urdf, n_links);
generate_load_regressor(path_to_urdf, n_links);


% Run tests
test_rb_inverse_dynamics(n_links)
test_base_params(n_links)


% Perform QR decompostion in order to get base parameters of the robot
include_motor_dynamics = 1;
[pi_lgr_base, baseQR] = base_params_qr(include_motor_dynamics, n_links);


% Estimate drive gains
m_load = 2.805;
path_to_unloaded_traj = 'ur-20_02_19_14harm50sec.csv';
path_to_loaded_traj = 'ur-20_02_19_14harm50secLoad.csv';
drive_gains = estimate_drive_gains(baseQR, 'PC-OLS', n_links, m_load,...
                                   path_to_loaded_traj, path_to_unloaded_traj);
% Or use those found in the paper by De Luca
% drive_gains = [14.87; 13.26; 11.13; 10.62; 11.03; 11.47]; 


% Estimate dynamic parameters
path_to_est_data = 'ur10_simulation_telemetry.csv';      idxs = [1, 390];
% path_to_data = 'ur-20_02_12-40sec_12harm.csv';    idxs = [500, 4460];    
% path_to_data = 'ur-20_02_05-20sec_8harm.csv';     idxs = [320, 2310];
% path_to_data = 'ur-20_02_12-50sec_12harm.csv';    idxs = [355, 5090];
sol = estimate_dynamic_params(path_to_est_data, idxs, ...
                              drive_gains, baseQR, 'PC-OLS', n_links);


% Validate estimated parameters
path_to_val_data = 'ur10_simulation_telemetry_.csv';     idxs = [1, 390];



rre = validate_dynamic_params(path_to_val_data , idxs, ...
                              drive_gains, baseQR, sol.pi_b, sol.pi_fr, n_links)












