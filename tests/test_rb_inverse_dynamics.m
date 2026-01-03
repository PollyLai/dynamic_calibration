function test_rb_inverse_dynamics(path_to_urdf, dof)

% path_to_urdf = 'ur10e.urdf';
ur10 = parse_urdf(path_to_urdf, dof);

rbt = importrobot(path_to_urdf);
rbt.DataFormat = 'column';
rbt.Gravity = [0 0 -9.81];


% for i = 1:dof
%     j_rbt  = rbt.Bodies{i}.Joint;
%     axis_rbt = j_rbt.JointAxis;
%     T_rbt    = j_rbt.JointToParentTransform;   % 4x4: parent link -> joint frame

%     % 你 parse_urdf 抓到的
%     j_xml  = ur10.robot.joint{i};
%     axis_xml = str2num(j_xml.axis.Attributes.xyz)';
%     rpy_xml  = str2num(j_xml.origin.Attributes.rpy);
%     xyz_xml  = str2num(j_xml.origin.Attributes.xyz)';

%     fprintf('Joint %d:\n', i);
%     fprintf('  rbt axis      = [%.3f %.3f %.3f]\n', axis_rbt);
%     fprintf('  urdf axis     = [%.3f %.3f %.3f]\n', axis_xml);
%     fprintf('  rbt origin xyz= [%.3f %.3f %.3f]\n', T_rbt(1:3,4));
%     fprintf('  urdf origin   = [%.3f %.3f %.3f]\n\n', xyz_xml);
% end
% % 
showdetails(rbt)
fprintf('=== COM 比對 ===\n');
for i = 1:dof
    body = rbt.Bodies{i};
    fprintf('Body %d (%s)\n', i, body.Name);
    fprintf('  ur10.r_com(:,%d) = [% .6f % .6f % .6f]\n', ...
        i, ur10.r_com(:,i));
    fprintf('  rbt COM         = [% .6f % .6f % .6f]\n\n', ...
        body.CenterOfMass);
end

% fprintf('=== Joint / Link 幾何 ===\n')
% % URDF joint axis / origin
% axis_urdf = str2num(ur10.robot.joint{1}.axis.Attributes.xyz).';
% org_urdf  = str2num(ur10.robot.joint{1}.origin.Attributes.xyz).';
% 
% % RBT joint axis / origin
% body = rbt.Bodies{1};
% axis_rbt = body.Joint.JointAxis(:);
% T_rbt    = body.Joint.JointToParentTransform;
% org_rbt  = tform2trvec(T_rbt).';
% 
% disp(table(axis_urdf, axis_rbt, org_urdf, org_rbt, ...
%     'VariableNames', {'axis_urdf','axis_rbt','origin_urdf','origin_rbt'}))
% 
% fprintf('\n=== Mass / COM 對比 ===\n')
% % URDF inertial
% link = ur10.robot.link{2};  % link{1} = base_link, link{2} = l_shank_1
% m_urdf   = str2double(link.inertial.mass.Attributes.value);
% com_urdf = str2num(link.inertial.origin.Attributes.xyz).';
% 
% % RBT
% m_rbt   = body.Mass;
% com_rbt = body.CenterOfMass(:);
% 
% disp(table(m_urdf, m_rbt, ...
%     'VariableNames', {'m_urdf','m_rbt'}))
% disp(table(com_urdf, com_rbt, ...
%     'VariableNames', {'com_urdf','com_rbt'}))
% 
% fprintf('\n=== Inertia 對比 ===\n')
% % URDF inertia (通常關於 COM)
% Ixx = str2double(link.inertial.inertia.Attributes.ixx);
% Ixy = str2double(link.inertial.inertia.Attributes.ixy);
% Ixz = str2double(link.inertial.inertia.Attributes.ixz);
% Iyy = str2double(link.inertial.inertia.Attributes.iyy);
% Iyz = str2double(link.inertial.inertia.Attributes.iyz);
% Izz = str2double(link.inertial.inertia.Attributes.izz);
% 
% I_urdf_COM = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
% 
% % 把 URDF 的 inertia 用平行軸轉成「關於 link frame 原點」
% p = com_urdf;
% S = [   0   -p(3)  p(2);
%       p(3)   0    -p(1);
%      -p(2)  p(1)   0  ];
% 
% I_urdf_about_link = I_urdf_COM - m_urdf * (S * S);
% 
% 
% % RBT inertia 是 [Ixx Iyy Izz Iyz Ixz Ixy]，關於 body 原點
% I6 = body.Inertia;
% I_rbt = [ I6(1) I6(6) I6(5);
%           I6(6) I6(2) I6(4);
%           I6(5) I6(4) I6(3) ];
% 
% disp('I_urdf_about_link ='); disp(I_urdf_about_link)
% disp('I_rbt (about body origin) ='); disp(I_rbt)
% disp('I_rbt - I_urdf_about_link ='); disp(I_rbt - I_urdf_about_link)
% 
% reshape(ur10.pi,[dof*10,1]);
% ur10.pi,[dof*10,1]

no_iter = 100;
for i = 1:no_iter
    % q_min = deg2rad([-30; -30; -30]);
    % q_max = deg2rad([ 30;  30;  30]);
    % q_min = ([-0.4363323  -0.3490659  -0.6981317   -1.0471976  -0.523599  -0.523599]');
    % q_max = ([0.4363319    1.0471976   0.5235988    0.5235988   0.523599   0.523599]');
    % q = q_min + (q_max - q_min).*rand(dof,1);
    % q_d = zeros(3,1);
    % q_2d = zeros(3,1);

    q = -2*pi + 4*pi*rand(dof,1);
    q_d = zeros(dof,1);
    q_2d = zeros(dof,1);
    % q = [0.7641]
    Ylgr = standard_regressor_UR10E(q,q_d,q_2d);

    % fprintf('q = [%.4f  %.4f  %.4f]\n\n', q(1), q(2), q(3));
    % fprintf('q = [%.4f]\n\n', q);

    tau_matlab = inverseDynamics(rbt,q,q_d,q_2d);
    tau_reg = Ylgr*reshape(ur10.pi,[dof*10,1]);
    % tau_manip = M_mtrx_fcn(q, ur10.pi(:))*q_2d + ...
    %             C_mtrx_fcn(q, q_d, ur10.pi(:))*q_d + ...
    %             G_vctr_fcn(q, ur10.pi(:));
    % Y_h_part = Ylgr(1, 7:9)   % 對應 h 的 3 個係數
    % Y_m_part = Ylgr(1, 10)    % 對應 m 的係數
    % disp('=== FULL (全部link都有質量) ===');
    % disp('tau_matlab ='); disp(tau_matlab);
    % disp('tau_reg    ='); disp(tau_reg);
    % disp('diff       ='); disp(tau_matlab - tau_reg);
    % fprintf('\n');
    
    % % 再看單一link的貢獻
    % fprintf('=== 單一 link 質量貢獻 (只保留一節) ===\n');
    % 
    % for link_id = 1:dof
    %     fprintf('\n--- Link %d (%s) ---\n', link_id, rbt.Bodies{link_id}.Name);
    % 
    %     % 只保留這一節的 pi，其它設 0
    %     pi_test = zeros(size(ur10.pi));
    %     pi_test(:, link_id) = ur10.pi(:, link_id);  % 保留第link_id欄
    % 
    %     tau_reg_link = Ylgr * pi_test(:);
    % 
    %     % 複製一個 rbt，只保留這一節有質量
    %     rbt_link = copy(rbt);
    %     for j = 1:dof
    %         if j ~= link_id
    %             rbt_link.Bodies{j}.Mass         = 0;
    %             rbt_link.Bodies{j}.Inertia      = zeros(1,6);
    %             rbt_link.Bodies{j}.CenterOfMass = [0 0 0];
    %         end
    %     end
    %     fprintf('  Masses in rbt_link: [');
    %     for j = 1:dof
    %         fprintf(' %.4f', rbt_link.Bodies{j}.Mass);
    %     end
    %     fprintf(' ]\n');
    % 
    %     tau_mlab_link = inverseDynamics(rbt_link, q, q_d, q_2d);
    % 
    %     disp('tau_matlab_link ='); disp(tau_mlab_link);
    %     disp('tau_reg_link    ='); disp(tau_reg_link);
    %     disp('diff            ='); disp(tau_mlab_link - tau_reg_link);
    % end
%   verifying if regressor is computed correctly
    diff_reg = (tau_matlab - tau_reg)
    % diff_manip = (tau_matlab - tau_manip);
    assert(norm(diff_reg) < 1e-8);
    % assert(norm(diff_manip) < 1e-8);
    % assert(norm(tau_matlab - tau_reg) < 1e-8);
    % assert(norm(tau_matlab - tau_manip) < 1e-8);
end

fprintf("Rigid Body Inverse Dynamics Test - OK!\n");