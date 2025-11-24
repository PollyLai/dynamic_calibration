clear all; close all; clc;


% Define path to a urdf file
addpath(genpath("C:\Users\polly\v1\dynamic_calibration_10"));
% path_to_urdf = 'urdf\fixed_bipedal.urdf';
% addpath(genpath('/home/ldsc/matlab_10/dynamic_calibration'));
path_to_urdf = 'test_inverted_10_no_fixed.urdf';
dof = 10;
% path_to_urdf = 'ur10e.urdf';
% dof = 6;


% Generate functions for dynamics based on Lagrange method
% Note that it might take some time
generate_rb_dynamics(path_to_urdf, dof);
generate_friction_eq();


% Generate regressors for inverse dynamics of the robot, friction and load
% Note that it might take some time
% generate_rb_regressor(path_to_urdf, dof);
% generate_load_regressor(path_to_urdf);


% % Run tests
test_rb_inverse_dynamics(path_to_urdf, dof)
test_base_params(path_to_urdf, dof)
% 
% 
% % Perform QR decompostion in order to get base parameters of the robot
% include_motor_dynamics = 1;
% [pi_lgr_base, baseQR] = base_params_qr(include_motor_dynamics);
% 
% 
% % Estimate drive gains
% % drive_gains = estimate_drive_gains(baseQR, 'PC-OLS');
% % Or use those found in the paper by De Luca
% % drive_gains = [14.87; 13.26; 11.13; 10.62; 11.03; 11.47]; 
% drive_gains = [1; 1; 1; 1; 1; 1]; 
% 
% 
% % ====================================
% % Estimate dynamic parameters
% path_to_est_data = 'ur10_simulation_telemetry_perfect_titleless.csv';      idxs = [1, 1200];
% % path_to_data = 'ur-20_02_12-40sec_12harm.csv';    idxs = [500, 4460];    
% % path_to_data = 'ur-20_02_05-20sec_8harm.csv';     idxs = [320, 2310];
% % path_to_data = 'ur-20_02_12-50sec_12harm.csv';    idxs = [355, 5090];
% sol = estimate_dynamic_params(path_to_est_data, idxs, ...
%                               drive_gains, baseQR, 'PC-OLS');
% 
% 
% % Validate estimated parameters
% path_to_val_data = 'ur10_simulation_telemetry_perfect_titleless.csv';     idxs = [1, 1200];
% 
% rre = validate_dynamic_params(path_to_val_data, idxs, ...
%                               drive_gains, baseQR, sol.pi_b, sol.pi_fr)
% % ===========================================================================
% 
% 
% 
% % 
% % 
% % % % === frictionless === %
% % path_to_est_data = 'ur10_simulation_telemetry_titleless.csv';     idxs = [1, 1249];
% % path_to_val_data = 'ur10_simulation_telemetry_titleless.csv';     idxs = [1, 1249];
% % drive_gains = [1; 1; 1; 1; 1; 1]; 
% % % drive_gains = estimate_drive_gains_frictionless(baseQR, 'PC-OLS');
% % 
% % sol = estimate_dynamic_params_frictionless(path_to_est_data, idxs, ...
% %                               drive_gains, baseQR, 'PC-OLS');
% % rre = validate_dynamic_params_frictionless(path_to_val_data, idxs, ...
% %                               drive_gains, baseQR, sol.pi_b)
% % 
% % 
% % 
% % 
% 
% writematrix([sol.pi_b; sol.pi_fr], 'output_csv\base_parameters.csv');
% 
% writematrix(baseQR.permutationMatrix, 'output_csv\permutationMatrix.csv');
% 
% 
% 
% 
% 
% 
% 
