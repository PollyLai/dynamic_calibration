function export_dynamics_to_yaml(filename)
% EXPORT_DYNAMICS_TO_YAML Export all identified dynamics parameters to a single YAML file
%
% Usage:
%   export_dynamics_to_yaml('robot_dynamics.yaml')
%
% This function runs the complete identification pipeline and exports:
%   - Base parameters (inertial parameters)
%   - Friction parameters  
%   - Drive gains
%   - Standard parameters (if PC-OLS is used)
%   - Statistical analysis (standard deviations, relative std)
%   - Validation results (RRE)
%   - Metadata (timestamps, methods used, data files)

    if nargin < 1
        filename = 'identified_dynamics.yaml';
    end
    
    fprintf('Starting dynamics identification and export...\n');
    
    % ============= Initialize and Setup =============
    % Clear workspace for clean run
    clearvars -except filename
    
    % Store metadata
    metadata = struct();
    metadata.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    metadata.matlab_version = version;
    
    % Define paths and parameters
    path_to_urdf = 'ur10e.urdf';
    metadata.urdf_file = path_to_urdf;
    
    % ============= Generate Dynamics Functions =============
    fprintf('Generating dynamics functions...\n');
    
    % Check if functions need to be generated
    if ~exist('autogen/M_mtrx_fcn.m', 'file')
        generate_rb_dynamics(path_to_urdf);
    end
    
    if ~exist('autogen/F_vctr_fcn.m', 'file')
        generate_friction_eq();
    end
    
    if ~exist('autogen/standard_regressor_UR10E.m', 'file')
        generate_rb_regressor(path_to_urdf);
    end
    
    % ============= Run Tests =============
    fprintf('Running tests...\n');
    test_rb_inverse_dynamics();
    test_base_params();
    
    % ============= QR Decomposition for Base Parameters =============
    fprintf('Computing base parameters via QR decomposition...\n');
    include_motor_dynamics = 1;
    [pi_lgr_base, baseQR] = base_params_qr(include_motor_dynamics);
    
    % Store QR decomposition info
    qr_info = struct();
    qr_info.number_of_base_parameters = baseQR.numberOfBaseParameters;
    qr_info.motor_dynamics_included = baseQR.motorDynamicsIncluded;
    
    % ============= Estimate Drive Gains =============
    fprintf('Estimating drive gains...\n');
    method_drive = 'PC-OLS';
    metadata.drive_gain_method = method_drive;
    
    try
        drive_gains = estimate_drive_gains(baseQR, method_drive);
    catch
        % Fallback to De Luca values if estimation fails
        fprintf('Using De Luca drive gains as fallback...\n');
        drive_gains = [14.87; 13.26; 11.13; 10.62; 11.03; 11.47];
        metadata.drive_gain_method = 'De_Luca_values';
    end
    
    % ============= Estimate Dynamic Parameters =============
    fprintf('Estimating dynamic parameters...\n');
    
    % Define trajectory data for estimation
    estimation_data = struct();
    estimation_data.file = 'ur10_simulation_telemetry_2.csv';
    estimation_data.indices = [100, 1400];
    metadata.estimation_data = estimation_data.file;
    metadata.estimation_indices = estimation_data.indices;
    
    % Estimation method
    method_dynamics = 'PC-OLS';
    metadata.dynamics_method = method_dynamics;
    
    % Perform estimation
    sol = estimate_dynamic_params(estimation_data.file, estimation_data.indices, ...
                                  drive_gains, baseQR, method_dynamics);
    
    % ============= Validate Parameters =============
    fprintf('Validating estimated parameters...\n');
    
    % Define validation data
    validation_data = struct();
    validation_data.file = 'ur10_simulation_telemetry_2.csv';
    validation_data.indices = [100, 1400];
    metadata.validation_data = validation_data.file;
    metadata.validation_indices = validation_data.indices;
    
    % Perform validation
    rre = validate_dynamic_params(validation_data.file, validation_data.indices, ...
                                  drive_gains, baseQR, sol.pi_b, sol.pi_fr);
    
    % ============= Parse URDF for additional info =============
    ur10 = parse_urdf(path_to_urdf);
    
    % ============= Create YAML Structure =============
    fprintf('Creating YAML structure...\n');
    
    % Open file for writing
    fid = fopen(filename, 'w');
    if fid == -1
        error('Could not open file %s for writing', filename);
    end
    
    % Write YAML header
    fprintf(fid, '# UR10E Robot Identified Dynamics Parameters\n');
    fprintf(fid, '# Generated: %s\n\n', metadata.timestamp);
    
    % Write metadata
    fprintf(fid, 'metadata:\n');
    fprintf(fid, '  timestamp: "%s"\n', metadata.timestamp);
    fprintf(fid, '  urdf_file: "%s"\n', metadata.urdf_file);
    fprintf(fid, '  estimation_method: "%s"\n', metadata.dynamics_method);
    fprintf(fid, '  drive_gain_method: "%s"\n', metadata.drive_gain_method);
    fprintf(fid, '  estimation_data: "%s"\n', metadata.estimation_data);
    fprintf(fid, '  estimation_indices: [%d, %d]\n', metadata.estimation_indices(1), metadata.estimation_indices(2));
    fprintf(fid, '  validation_data: "%s"\n', metadata.validation_data);
    fprintf(fid, '  validation_indices: [%d, %d]\n', metadata.validation_indices(1), metadata.validation_indices(2));
    fprintf(fid, '\n');
    
    % Write QR decomposition info
    fprintf(fid, 'base_parameters_info:\n');
    fprintf(fid, '  number_of_base_parameters: %d\n', qr_info.number_of_base_parameters);
    fprintf(fid, '  motor_dynamics_included: %s\n', bool2str(qr_info.motor_dynamics_included));
    fprintf(fid, '\n');
    
    % Write drive gains
    fprintf(fid, 'drive_gains:\n');
    for i = 1:length(drive_gains)
        fprintf(fid, '  joint_%d: %.6f\n', i, drive_gains(i));
    end
    fprintf(fid, '\n');
    
    % Write base parameters
    fprintf(fid, 'base_parameters:\n');
    fprintf(fid, '  dimension: %d\n', length(sol.pi_b));
    fprintf(fid, '  values:\n');
    for i = 1:length(sol.pi_b)
        fprintf(fid, '    - index: %d\n', i);
        fprintf(fid, '      value: %.8e\n', sol.pi_b(i));
        if isfield(sol, 'std') && i <= length(sol.pi_b)
            fprintf(fid, '      std: %.8e\n', sol.std(i));
            fprintf(fid, '      rel_std_percent: %.2f\n', sol.rel_std(i));
        end
    end
    fprintf(fid, '\n');
    
    % Write friction parameters
    fprintf(fid, 'friction_parameters:\n');
    fprintf(fid, '  dimension: %d\n', length(sol.pi_fr));
    fprintf(fid, '  # Format: [viscous, coulomb, offset] for each joint\n');
    for i = 1:6
        idx_base = (i-1)*3;
        fprintf(fid, '  joint_%d:\n', i);
        fprintf(fid, '    viscous: %.8e\n', sol.pi_fr(idx_base + 1));
        fprintf(fid, '    coulomb: %.8e\n', sol.pi_fr(idx_base + 2));
        fprintf(fid, '    offset: %.8e\n', sol.pi_fr(idx_base + 3));
        if isfield(sol, 'std')
            std_idx = length(sol.pi_b) + idx_base;
            fprintf(fid, '    viscous_std: %.8e\n', sol.std(std_idx + 1));
            fprintf(fid, '    coulomb_std: %.8e\n', sol.std(std_idx + 2));
            fprintf(fid, '    offset_std: %.8e\n', sol.std(std_idx + 3));
        end
    end
    fprintf(fid, '\n');
    
    % Write standard parameters if available (from PC-OLS)
    if isfield(sol, 'pi_s')
        fprintf(fid, 'standard_parameters:\n');
        fprintf(fid, '  dimension: %d\n', length(sol.pi_s));
        fprintf(fid, '  # Inertial parameters in standard form [Ixx, Ixy, Ixz, Iyy, Iyz, Izz, mx, my, mz, m, Im]\n');
        for link = 1:6
            fprintf(fid, '  link_%d:\n', link);
            base_idx = (link-1)*11;
            if base_idx + 11 <= length(sol.pi_s)
                fprintf(fid, '    Ixx: %.8e\n', sol.pi_s(base_idx + 1));
                fprintf(fid, '    Ixy: %.8e\n', sol.pi_s(base_idx + 2));
                fprintf(fid, '    Ixz: %.8e\n', sol.pi_s(base_idx + 3));
                fprintf(fid, '    Iyy: %.8e\n', sol.pi_s(base_idx + 4));
                fprintf(fid, '    Iyz: %.8e\n', sol.pi_s(base_idx + 5));
                fprintf(fid, '    Izz: %.8e\n', sol.pi_s(base_idx + 6));
                fprintf(fid, '    mx: %.8e\n', sol.pi_s(base_idx + 7));
                fprintf(fid, '    my: %.8e\n', sol.pi_s(base_idx + 8));
                fprintf(fid, '    mz: %.8e\n', sol.pi_s(base_idx + 9));
                fprintf(fid, '    mass: %.8e\n', sol.pi_s(base_idx + 10));
                if include_motor_dynamics && base_idx + 11 <= length(sol.pi_s)
                    fprintf(fid, '    Im: %.8e\n', sol.pi_s(base_idx + 11));
                end
            end
        end
        fprintf(fid, '\n');
    end
    
    % Write validation results
    fprintf(fid, 'validation_results:\n');
    fprintf(fid, '  relative_residual_error_percent:\n');
    for i = 1:length(rre)
        fprintf(fid, '    joint_%d: %.2f\n', i, rre(i));
    end
    fprintf(fid, '  mean_rre: %.2f\n', mean(rre));
    fprintf(fid, '  max_rre: %.2f\n', max(rre));
    fprintf(fid, '\n');
    
    % Write joint information from URDF
    fprintf(fid, 'joint_info:\n');
    for i = 1:6
        fprintf(fid, '  joint_%d:\n', i);
        fprintf(fid, '    name: "%s"\n', ur10.robot.joint{i}.Attributes.name);
        fprintf(fid, '    type: "%s"\n', ur10.robot.joint{i}.Attributes.type);
        fprintf(fid, '    axis: [%s]\n', ur10.robot.joint{i}.axis.Attributes.xyz);
        fprintf(fid, '    origin_xyz: [%s]\n', ur10.robot.joint{i}.origin.Attributes.xyz);
        fprintf(fid, '    origin_rpy: [%s]\n', ur10.robot.joint{i}.origin.Attributes.rpy);
    end
    
    % Close file
    fclose(fid);
    
    fprintf('Successfully exported dynamics to: %s\n', filename);
    fprintf('Export complete!\n');
    
    % Display summary
    fprintf('\n========== EXPORT SUMMARY ==========\n');
    fprintf('Output file: %s\n', filename);
    fprintf('Base parameters: %d\n', length(sol.pi_b));
    fprintf('Friction parameters: %d\n', length(sol.pi_fr));
    fprintf('Mean validation RRE: %.2f%%\n', mean(rre));
    fprintf('====================================\n');
    
end

% Helper function to convert boolean to string
function str = bool2str(val)
    if val
        str = 'true';
    else
        str = 'false';
    end
end