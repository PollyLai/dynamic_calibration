function rre = validate_dynamic_params_frictionless(path_to_data, idx, drvGains, baseQR, pi_b, pi_fr)
% ---------------------------------------------------------------------
% Function perform validation of a given parameters on a given trajectory
% 
% Inputs:
%   path_to_data - measured data from the robot
%   idx - specifies indeces for data to be used in estimation. Used
%         to remove garbage data
%   drvGains - drive gains
%   baseQR - QR decomposition of the observation matrix used for 
%            calculatung base regressor
%   pi_b - base parameters of the robot
%   pi_fr - friction parameters of the robot
%
% Ouputs:
%   rre - relative residual error
% -----------------------------------------------------------------------
% Load, parse and filter data
vldtnTrjctry = parseURData(path_to_data, idx(1), idx(2));
vldtnTrjctry = filterData(vldtnTrjctry);

% Predict torques based on position, velocity and acceelration using
% given parameters
tau_msrd = []; 
tau_pred = [];
for i = 1:length(vldtnTrjctry.t)
    qi = vldtnTrjctry.q(i,:)';
    qdi = vldtnTrjctry.qd_fltrd(i,:)';
    q2di = vldtnTrjctry.q2d_est(i,:)';
    if baseQR.motorDynamicsIncluded
        Yi = regressorWithMotorDynamics(qi, qdi, q2di);
    else 
        Yi = standard_regressor_UR10E(qi, qdi, q2di);
    end
    Ybi = Yi*baseQR.permutationMatrix(:,1:baseQR.numberOfBaseParameters);
    % Yfrctni = frictionRegressor(qdi);
    
    % tau_msrd = horzcat(tau_msrd, diag(drvGains)*vldtnTrjctry.i(i,:)');
    tau_pred = horzcat(tau_pred, Ybi*pi_b);
    % tau_msrd = horzcat(tau_msrd, vldtnTrjctry.i(i,:)');
    tau_msrd = horzcat(tau_msrd, vldtnTrjctry.i_fltrd(i,:)');
end

% Calculate relative residual error
rre = zeros(6,1);
for i = 1:6
   rre(i) = 100*norm(tau_msrd(i,:) - tau_pred(i,:))/norm(tau_msrd(i,:)); 
end

% Plot measured torques against predicted ones
for i = 1:6
    f = figure('Theme', 'light'); % Forces light theme
    f.Color = [1, 1, 1]; % White background
    hold on
    % Example: Changing theme programmatically
    theme(f, 'light'); % Applies dark theme to a specific figure
    plot(vldtnTrjctry.t, tau_msrd(i,:), 'LineWidth', 1.2)
    plot(vldtnTrjctry.t, tau_pred(i,:), 'LineWidth', 1)
    ylabel('\tau, Nm')
    xlabel('t, sec')
    legend('measured', 'predicted')
    grid on
end


% % === Plots ===
% figure; 
% plot(vldtnTrjctry.t, vldtnTrjctry.qd, '--'); hold on;
% plot(vldtnTrjctry.t, vldtnTrjctry.qd_fltrd, 'LineWidth', 1.2);
% xlabel('t (s)'); ylabel('qd (rad/s)');
% title('Joint velocities: raw vs. filtered');
% legend('qd (raw)','qd\_fltrd','Location','best'); grid on;
% 
% figure;
% plot(vldtnTrjctry.t, vldtnTrjctry.q, '--'); hold on;
% plot(vldtnTrjctry.t, vldtnTrjctry.q_fltrd, 'LineWidth', 1.2);
% xlabel('t (s)'); ylabel('q (rad)');
% title('Joint positions: raw vs. filtered');
% legend('q (raw)','q\_fltrd','Location','best'); grid on;
% 







