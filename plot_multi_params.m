function rre = plot_multi_params(path_to_data, idx, baseQR, drive_gains, varargin)

options.plot_measured = true;
options.line_width_measured = 0.8;
options.line_width_predicted = 1.0;
options.colors = ['b', 'r', 'g', 'm', 'c', 'k']; 
param_sets = [];
arg_idx = 1;
param_sets = varargin{1};
arg_idx = 2;

for i = 1:length(param_sets)
    param_sets(i).drvGains = param_sets(i).drvGains';
end

% Load, parse and filter data
vldtnTrjctry = parseURData(path_to_data, idx(1), idx(2));
vldtnTrjctry = filterData(vldtnTrjctry);

n_sets = length(param_sets);
tau_measured = [];
tau_predicted = cell(n_sets, 1);
rre = zeros(6, n_sets);

for i = 1:length(vldtnTrjctry.t)
    tau_measured = horzcat(tau_measured, diag(drive_gains)*vldtnTrjctry.i_fltrd(i,:)');
end

% tau_measured = drive_gains .* tau_measured;

for set_idx = 1:n_sets
    pi_b = param_sets(set_idx).pi_b;
    pi_fr = param_sets(set_idx).pi_fr;
    drvGains = param_sets(set_idx).drvGains;
    tau_pred_current = [];
    
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
        Yfrctni = frictionRegressor(qdi);
        
        if size(pi_b, 2) > 1
            pi_b = pi_b';
        end
        if size(pi_fr, 2) > 1
            pi_fr = pi_fr';
        end
        
        regressor_combined = [Ybi Yfrctni];
        params_combined = [pi_b; pi_fr];
        
        tau_pred_current = horzcat(tau_pred_current, regressor_combined * params_combined);
    end
    
    % tau_predicted{set_idx} = drvGains .* tau_pred_current;
    tau_predicted{set_idx} = tau_pred_current;
    
    for joint = 1:6
        rre(joint, set_idx) = 100*norm(tau_measured(joint,:) - tau_predicted{set_idx}(joint,:))/norm(tau_measured(joint,:));
    end
end

% Display RRE results
fprintf('\n=== Relative Residual Error (RRE) Results ===\n');
fprintf('Joint');
for set_idx = 1:n_sets
    fprintf('\t%s', param_sets(set_idx).label);
end
fprintf('\n');

for joint = 1:6
    fprintf('J%d', joint);
    for set_idx = 1:n_sets
        fprintf('\t\t%.2f%%', rre(joint, set_idx));
    end
    fprintf('\n');
end

joint_names = {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'};
figure('Name', 'Torque Validation - All Joints', ...
       'Position', [100, 100, 1200, 800]);

for joint = 1:6
    subplot(2, 3, joint);
    hold on;
    grid on;
    if options.plot_measured
        plot(vldtnTrjctry.t, tau_measured(joint,:), 'w-', ...
             'LineWidth', options.line_width_measured, ...
             'DisplayName', 'Measured');
    end
    
    for set_idx = 1:n_sets
        color_idx = mod(set_idx - 1, length(options.colors)) + 1;
        plot(vldtnTrjctry.t, tau_predicted{set_idx}(joint,:), ...
             [options.colors(color_idx) '--'], ...
             'LineWidth', options.line_width_predicted, ...
             'DisplayName', sprintf('%s (%.2f%%)', param_sets(set_idx).label, rre(joint, set_idx)));
    end
    
    ylabel('Torque (Nm)', 'FontSize', 10);
    xlabel('Time (s)', 'FontSize', 10);
    title(sprintf('%s', joint_names{joint}), 'FontSize', 12, 'FontWeight', 'bold');
    if joint == 1
        legend('Location', 'best', 'FontSize', 9);
    end
    
    rre_text = '';
    for set_idx = 1:n_sets
        if set_idx == 1
            rre_text = sprintf('RRE: %s %.1f%%', param_sets(set_idx).label, rre(joint, set_idx));
        else
            rre_text = [rre_text sprintf('\n%s %.1f%%', param_sets(set_idx).label, rre(joint, set_idx))];
        end
    end
    
    text(0.98, 0.98, rre_text, 'Units', 'normalized', ...
         'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', ...
         'BackgroundColor', 'white', 'EdgeColor', 'black', ...
         'FontSize', 8);
end

sgtitle('Measured vs Predicted Torques - All Joints', 'FontSize', 16, 'FontWeight', 'bold');
figure('Name', 'RRE Summary', 'Position', [200, 200, 1000, 600]);

subplot(1, 2, 1);
bar(rre');
xlabel('Parameter Set', 'FontSize', 12);
ylabel('Relative Residual Error (%)', 'FontSize', 12);
title('RRE Comparison by Parameter Set', 'FontSize', 14);
legend(joint_names, 'Location', 'best', 'FontSize', 10);
grid on;
set(gca, 'XTickLabel', {param_sets.label});

subplot(1, 2, 2);
bar(rre);
xlabel('Joint', 'FontSize', 12);
ylabel('Relative Residual Error (%)', 'FontSize', 12);
title('RRE Comparison by Joint', 'FontSize', 14);
legend({param_sets.label}, 'Location', 'best', 'FontSize', 10);
grid on;
set(gca, 'XTickLabel', joint_names);

if n_sets == 1
    rre = rre(:, 1);
end

end