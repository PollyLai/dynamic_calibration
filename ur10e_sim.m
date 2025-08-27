% Load robot model
robot = importrobot('ur10e.urdf');
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81];

% Load trajectory coefficient data
load('ptrnSrch_N7T25QR.mat'); % Replace with your filename
whos

t = traj_par.t;
wf = traj_par.wf;
N = traj_par.N;         % Number of harmonics
n_joints = 6;           % For UR10e

q_des = zeros(n_joints, length(t));
for j = 1:n_joints
    for k = 1:N
        q_des(j, :) = q_des(j, :) + ...
            a(j, k) * sin(wf*k*t) + ...
            b(j, k) * cos(wf*k*t);
    end

end


ee_points = zeros(3, length(t));
for i = 1:length(t)
    T = getTransform(robot, q_des(:,i), 'tool0'); % Use your EE link name
    ee_points(:,i) = T(1:3,4);
end


% Load robot model
robot = importrobot('ur10e.urdf');
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81];

% Load trajectory coefficient data
load('ptrnSrch_N7T25QR.mat'); % Replace with your filename
whos

t = traj_par.t;
wf = traj_par.wf;
N = traj_par.N;         % Number of harmonics
n_joints = 6;           % For UR10e

q_des = zeros(n_joints, length(t));
for j = 1:n_joints
    for k = 1:N
        q_des(j, :) = q_des(j, :) + ...
            a(j, k) * sin(wf*k*t) + ...
            b(j, k) * cos(wf*k*t);
    end

end


ee_points = zeros(3, length(t));
for i = 1:length(t)
    T = getTransform(robot, q_des(:,i), 'tool0'); % Use your EE link name
    ee_points(:,i) = T(1:3,4);
end


figure;
hold on;

% % Plot the 3D trajectory
% plot3(ee_points(1,:), ee_points(2,:), ee_points(3,:), 'LineWidth', 2);

% % Animate the robot along the trajectory
% for idx = 1:round(length(t)/100):length(t)
%     show(robot, q_des(:,idx), 'PreservePlot', false, 'Frames', 'off');
%     drawnow;
% end


% % Add colorbar and labels as in the example
% colormap('cool');
% cb = colorbar;
% cb.Ticks = [0 1];
% cb.TickLabels = {'Start','End'};
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('3D Trajectory with Robot');
% grid on; axis equal;
% view(3);

set(gcf,'Color','w');

% Create a color map (blue to red using 'cool' as example)
n_points = length(t);
cmap = colormap('cool');            % Get the colormap
n_map = size(cmap,1);
color_idx = round(linspace(1, n_map, n_points));

% Plot trajectory as colored segments
for i = 1:(n_points-1)
    this_color = cmap(color_idx(i),:);
    line(ee_points(1,i:i+1), ee_points(2,i:i+1), ee_points(3,i:i+1), ...
         'Color', this_color, 'LineWidth', 2);
end

% Animate robot along trajectory (unchanged)
for idx = 1:round(length(t)/100):length(t)
    show(robot, q_des(:,idx), 'PreservePlot', false, 'Frames', 'off');
    drawnow;
end

% Colorbar and labels
colormap('cool');
cb = colorbar;
cb.Ticks = [0 1];
cb.TickLabels = {'Start', 'End'};
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Trajectory with Robot (Color Gradient)');
grid on; axis equal; view(3);

