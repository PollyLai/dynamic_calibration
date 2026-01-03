function robot = parse_urdf(file, dof)
% Loading file from urdf
% The function is tailored for UR robots: serial robots with 6 n_links
% Modify it if you have  a robot with different degrees of freedom
robot = xml2struct(file);
% robot.robot.joint = {robot.robot.joint};
% Extracting parameters of the robot
for i = 1:dof
    disp(i);
    % axis of rotation of a joint i in coordinate system of joint i    
    axis_of_rot = str2num(robot.robot.joint{i}.axis.Attributes.xyz)';
    % mass of link (i+1) because joint i rotates link (i+1) as the numbering of
    % links starts from base link that it not moving
    link_mass = str2double(robot.robot.link{i+1}.inertial.mass.Attributes.value);
    % poistion of the com in frame attached to link
    com_pos = str2num(robot.robot.link{i+1}.inertial.origin.Attributes.xyz)';
    com_vec2mat = vec2skewSymMat(com_pos);
    % inertial parameters of the link expressed in coordinate system attached
    % the center of mass.
    ixx = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.ixx);
    ixy = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.ixy);
    ixz = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.ixz);
    iyy = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.iyy);
    iyz = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.iyz);
    izz = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.izz);
    % the inertia tensor wrt the frame oriented as the body frame and with the
    % origin in the COM
    link_inertia = [ixx, ixy, ixz; ixy, iyy iyz; ixz, iyz, izz];
    % manipulator regressor                               
    robot.m(i) = link_mass;
    disp(robot.m(i));
    robot.k(:,i) = axis_of_rot;
    robot.r_com(:,i) = com_pos;
    robot.I(:,:,i) = link_inertia;
    robot.h(:,i) = link_mass*com_pos;
    robot.I_vec(:,i) = inertiaMatrix2Vector(link_inertia - ...
                            link_mass*com_vec2mat*com_vec2mat);
    % robot.I_vec(:,i) = inertiaMatrix2Vector(link_inertia);
    % robot.I_vec(:,i) = inertiaMatrix2Vector(link_inertia + ...
    %                         link_mass*com_vec2mat*com_vec2mat);
    robot.pi(:,i) = [robot.I_vec(:,i); robot.h(:,i); robot.m(i)];
    % disp(robot.pi(:,i));

end

% --- 新增：合併第 11 與 12 軸到第 10 軸 ---
% 1. 取得 Fixed Joints 的位移 (從 URDF 內容中讀取)
% R_Ankle_Pitch (Joint 11): Shank -> Ankle
p_10_11 = str2num(robot.robot.joint{11}.origin.Attributes.xyz)'; 
% R_Ankle_Roll (Joint 12): Ankle -> Foot
p_11_12 = str2num(robot.robot.joint{12}.origin.Attributes.xyz)'; 

% 2. 取得 Fixed Links 的物理參數 (Link 11 是 l_ankle, Link 12 是 r_ankle, Link 13 是 r_foot)
% 注意：XML 解析後的索引可能依序為 link{11}, link{12}, link{13}，請根據你的 XML 結構確認
m11 = str2double(robot.robot.link{11+1}.inertial.mass.Attributes.value);
r11 = str2num(robot.robot.link{11+1}.inertial.origin.Attributes.xyz)';
m12 = str2double(robot.robot.link{12+1}.inertial.mass.Attributes.value);
r12 = str2num(robot.robot.link{12+1}.inertial.origin.Attributes.xyz)';

% 3. 將 m11, m12 的質心座標轉換回第 10 軸座標系 (Link 10 Frame)
% r_11_in_10 = p_10_11 + r11
% r_12_in_10 = p_10_11 + p_11_12 + r12
r11_fixed = p_10_11 + r11;
r12_fixed = p_10_11 + p_11_12 + r12;

% 4. 更新第 10 軸的質量與質心矩 (Static Moment h = mass * com)
m10_old = robot.m(10);
h10_old = robot.h(:,10);

m10_new = m10_old + m11 + m12;
h10_new = h10_old + m11*r11_fixed + m12*r12_fixed;

% 5. 寫回 robot 結構體
robot.m(10) = m10_new;
robot.h(:,10) = h10_new;
robot.r_com(:,10) = h10_new / m10_new; % 更新後的等效質心

% 6. 更新參數向量 pi (重心項)
robot.pi(7:10, 10) = [robot.h(:,10); robot.m(10)];

% --- 慣性張量 (I) 的合併 (平行軸定理) ---
% 如果你也需要鑑定慣性項，需要將 I11, I12 移軸到 Link 10 座標系再相加
% 但對於「靜態」或「低速」鑑定，質量與質心的合併是最關鍵的。