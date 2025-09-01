function data = filterData(data)
% ---------------------------------------------------------------------
% The function filters UR10E data for identification purposes.
% It filters generilized velocities and current, as well as estimates
% acceleration based on velocities by means of numerical differentiation
% with central difference scheme. The function also plots filtered data
% against unfileterd for visual inspection of the quality of filtering
% Input:
%   data - structure which has data.t, data.q, data.qd, data.i
% Ouput:
%   data - the same structure but with more elements, that are
%          data.qd_fltrd - filtered velocities, data.i_fltrd - 
%          filtered currents, data.q2d_est - estimated accelerations
% ---------------------------------------------------------------------

% data = parseURData('ur-20_02_05-20sec_8harm.csv', 320, 2310);

% ---------------------------------------------------------------------
% Filtering Velocities
% ---------------------------------------------------------------------
% Design filter
vel_filt = designfilt('lowpassiir','FilterOrder',5, ...
        'HalfPowerFrequency',0.15,'DesignMethod','butter');

data.qd_fltrd = zeros(size(data.qd));
for i = 1:6
    data.qd_fltrd(:,i) = filtfilt(vel_filt,data.qd(:,i));
end

% ------------------------------------------------------------------------
% Estimating accelerations
% ------------------------------------------------------------------------
% Three point central difference
data.q2d_est = zeros(size(data.qd_fltrd));
for i = 2:length(data.qd_fltrd)-1
   dlta_qd_fltrd =  data.qd_fltrd(i+1,:) -  ...
                        data.qd_fltrd(i-1,:);
   dlta_t_msrd = data.t(i+1) - data.t(i-1);
   data.q2d_est(i,:) = dlta_qd_fltrd/dlta_t_msrd;
end

% Zeros phase filtering acceleration obtained by finite difference
accel_filt = designfilt('lowpassiir','FilterOrder',5, ...
        'HalfPowerFrequency',0.15,'DesignMethod','butter');
for i = 1:6
    data.q2d_est(:,i) = filtfilt(accel_filt,data.q2d_est(:,i));
end

% ------------------------------------------------------------------------
% Filtering current
% ------------------------------------------------------------------------
% Zeros phase filtering acceleration obtained by finite difference
curr_filt = designfilt('lowpassiir','FilterOrder',5, ...
        'HalfPowerFrequency',0.20,'DesignMethod','butter');

for i = 1:6
    data.i_fltrd(:,i) = filtfilt(curr_filt,data.i(:,i));
end

% -----------------------------------------------------------------------
% Filtering desired current and desired torque
% -----------------------------------------------------------------------
for i = 1:6
    data.i_des_fltrd(:,i) = filtfilt(curr_filt,data.i_des(:,i));
end

for i = 1:6
    data.tau_des_fltrd(:,i) = filtfilt(curr_filt,data.tau_des(:,i));
end



% Functions for plotting
function plotVelocity(trj)
    figure
    plot(trj.t, trj.qd)
    hold on
    plot(trj.t, trj.qd_fltrd)
end

function plotCurrent(trj)
    figure
    plot(trj.t, trj.i)
    hold on
    plot(trj.t, trj.i_fltrd)
end

function plotAcceleration(trj)
    figure
    plot(trj.t, trj.q2d_est)
end

end
% function data = filterData(data)
% % ---------------------------------------------------------------------
% % The function filters UR10E data for identification purposes.
% % It filters generilized velocities and current, as well as estimates
% % acceleration based on velocities by means of numerical differentiation
% % with central difference scheme. The function also plots filtered data
% % against unfileterd for visual inspection of the quality of filtering
% % Input:
% %   data - structure which has data.t, data.q, data.qd, data.i
% % Ouput:
% %   data - the same structure but with more elements, that are
% %          data.qd_fltrd - filtered velocities, data.i_fltrd -
% %          filtered currents, data.q2d_est - estimated accelerations
% %          (plus PSD/time plots for raw vs filtered)
% % ---------------------------------------------------------------------

% % ---- sampling info (for PSD labeling only) ----
% dt = diff(data.t(:));
% dt = dt(dt > 0);
% Fs = 1/median(dt);           % Hz (assume near-uniform)
% Fnyq = Fs/2;

% % fc_vel_Hz = 0.15 * Fnyq;     % your original normalized cutoffs -> Hz
% % fc_cur_Hz = 0.20 * Fnyq;
% % ====== 自動挑 cutoff：每軸用 qd 的 PSD 抓任務頻率，再放寬 1.3× ======
% Fsafe = 0.45*Fnyq;                    % 離 Nyquist 遠一點，避免靠邊
% fc_vel = zeros(1,6); fc_cur = zeros(1,6);
% for j = 1:6
%     x = data.qd(:,j) - mean(data.qd(:,j));    % 去 DC 比較穩
%     [P,f] = pwelch(x, [], [], [], Fs);        % Welch PSD
%     if numel(f) > 1
%         % 忽略 DC，抓最大能量的頻率當作任務上緣
%         [~,k] = max(P(2:end)); k = k+1;
%         f_task = f(k);
%     else
%         f_task = 0.5; % fallback
%     end
%     % 速度/加速度的 cutoff：1.3× 任務上緣，並加上界線
%     fc_vel(j) = min(max(1.3*f_task, 0.3), Fsafe);
%     % 電流/期望量給得略高一些（不想過度磨平）
%     fc_cur(j) = min(max(1.2*fc_vel(j), 0.5), Fsafe);

%     fprintf('Joint %d: fc_vel=%.2f Hz, fc_cur=%.2f Hz (task≈%.2f Hz)\n', ...
%              j, fc_vel(j), fc_cur(j), f_task);
% end

% % ====== 用「Hz」設計每軸濾波器（取代原本 0.15/0.20 的固定值）======
% vel_filt = cell(1,6);  curr_filt = cell(1,6);
% for j = 1:6
%     vel_filt{j} = designfilt('lowpassiir','FilterOrder',5, ...
%         'HalfPowerFrequency', fc_vel(j), 'SampleRate', Fs, 'DesignMethod','butter');
%     curr_filt{j} = designfilt('lowpassiir','FilterOrder',5, ...
%         'HalfPowerFrequency', fc_cur(j), 'SampleRate', Fs, 'DesignMethod','butter');
% end

% % 速度濾波
% data.qd_fltrd = zeros(size(data.qd));
% for j = 1:6
%     data.qd_fltrd(:,j) = filtfilt(vel_filt{j}, data.qd(:,j));
% end

% % 中央差分 -> 加速度 -> 再用同一顆 vel_filt 濾一次
% N = size(data.qd_fltrd,1);
% data.q2d_raw = zeros(size(data.qd_fltrd));
% for k = 2:N-1
%    data.q2d_raw(k,:) = (data.qd_fltrd(k+1,:) - data.qd_fltrd(k-1,:)) / (data.t(k+1)-data.t(k-1));
% end
% data.q2d_raw(1,:)   = (data.qd_fltrd(2,:)   - data.qd_fltrd(1,:))   / (data.t(2)-data.t(1));
% data.q2d_raw(end,:) = (data.qd_fltrd(end,:) - data.qd_fltrd(end-1,:)) / (data.t(end)-data.t(end-1));

% data.q2d_est = zeros(size(data.q2d_raw));
% for j = 1:6
%     data.q2d_est(:,j) = filtfilt(vel_filt{j}, data.q2d_raw(:,j));
% end

% % 電流/期望量濾波
% data.i_fltrd = zeros(size(data.i));
% for j = 1:6, data.i_fltrd(:,j) = filtfilt(curr_filt{j}, data.i(:,j)); end
% if isfield(data,'i_des')
%     data.i_des_fltrd = zeros(size(data.i_des));
%     for j = 1:6, data.i_des_fltrd(:,j) = filtfilt(curr_filt{j}, data.i_des(:,j)); end
% end
% if isfield(data,'tau_des')
%     data.tau_des_fltrd = zeros(size(data.tau_des));
%     for j = 1:6, data.tau_des_fltrd(:,j) = filtfilt(curr_filt{j}, data.tau_des(:,j)); end
% end

% % ---------------------------------------------------------------------
% % Filtering Velocities
% % ---------------------------------------------------------------------
% % Design filter (normalized, same as your original)
% vel_filt = designfilt('lowpassiir','FilterOrder',5, ...
%         'HalfPowerFrequency',0.15,'DesignMethod','butter');

% data.qd_fltrd = zeros(size(data.qd));
% for j = 1:6
%     data.qd_fltrd(:,j) = filtfilt(vel_filt, data.qd(:,j));
% end

% % ---------------------------------------------------------------------
% % Estimating accelerations (central difference on filtered velocities)
% % ---------------------------------------------------------------------
% % Three point central difference (save raw central-diff before filtering)
% N = size(data.qd_fltrd,1);
% data.q2d_raw = zeros(size(data.qd_fltrd));    % <-- raw central-diff (unfiltered)
% for k = 2:N-1
%    dlta_qd_fltrd =  data.qd_fltrd(k+1,:) - data.qd_fltrd(k-1,:);
%    dlta_t_msrd   =  data.t(k+1) - data.t(k-1);
%    data.q2d_raw(k,:) = dlta_qd_fltrd / dlta_t_msrd;
% end
% % one-sided at edges (avoid zeros)
% data.q2d_raw(1,:) = (data.qd_fltrd(2,:) - data.qd_fltrd(1,:)) / (data.t(2)-data.t(1));
% data.q2d_raw(end,:) = (data.qd_fltrd(end,:) - data.qd_fltrd(end-1,:)) / (data.t(end)-data.t(end-1));

% % Zero-phase filtering acceleration obtained by finite difference
% accel_filt = designfilt('lowpassiir','FilterOrder',5, ...
%         'HalfPowerFrequency',0.15,'DesignMethod','butter');

% data.q2d_est = zeros(size(data.q2d_raw));
% for j = 1:6
%     data.q2d_est(:,j) = filtfilt(accel_filt, data.q2d_raw(:,j));
% end

% % ---------------------------------------------------------------------
% % Filtering current (and desired signals if present)
% % ---------------------------------------------------------------------
% curr_filt = designfilt('lowpassiir','FilterOrder',5, ...
%         'HalfPowerFrequency',0.20,'DesignMethod','butter');

% data.i_fltrd = zeros(size(data.i));
% for j = 1:6
%     data.i_fltrd(:,j) = filtfilt(curr_filt, data.i(:,j));
% end

% if isfield(data,'i_des')
%     data.i_des_fltrd = zeros(size(data.i_des));
%     for j = 1:6
%         data.i_des_fltrd(:,j) = filtfilt(curr_filt, data.i_des(:,j));
%     end
% end

% if isfield(data,'tau_des')
%     data.tau_des_fltrd = zeros(size(data.tau_des));
%     for j = 1:6
%         data.tau_des_fltrd(:,j) = filtfilt(curr_filt, data.tau_des(:,j));
%     end
% end

% % % ========================== PLOTTING ==========================
% % % 時域：raw vs filtered
% % plotMatrixTime(data.t, data.qd,        data.qd_fltrd, 'Velocity qd — raw vs filtered', 'rad/s');
% % plotMatrixTime(data.t, data.q2d_raw,   data.q2d_est,  'Acceleration (central diff) — raw vs filtered', 'rad/s^2');
% % plotMatrixTime(data.t, data.i,         data.i_fltrd,  'Current i — raw vs filtered', 'A');

% % if isfield(data,'i_des')
% %     plotMatrixTime(data.t, data.i_des, data.i_des_fltrd, 'Desired current i_{des} — raw vs filtered', 'A');
% % end
% % if isfield(data,'tau_des')
% %     plotMatrixTime(data.t, data.tau_des, data.tau_des_fltrd, 'Desired torque \tau_{des} — raw vs filtered', 'Nm');
% % end

% % % 頻域（PSD）：raw vs filtered（Welch），並標出 cutoff（Hz）
% % plotMatrixPSD(data.qd,      data.qd_fltrd,  Fs, fc_vel_Hz, 'Velocity qd — PSD (raw vs filtered)');
% % plotMatrixPSD(data.q2d_raw, data.q2d_est,   Fs, fc_vel_Hz, 'Acceleration — PSD (raw vs filtered)');
% % plotMatrixPSD(data.i,       data.i_fltrd,   Fs, fc_cur_Hz, 'Current i — PSD (raw vs filtered)');

% % if isfield(data,'i_des')
% %     plotMatrixPSD(data.i_des, data.i_des_fltrd, Fs, fc_cur_Hz, 'Desired current i_{des} — PSD (raw vs filtered)');
% % end
% % if isfield(data,'tau_des')
% %     plotMatrixPSD(data.tau_des, data.tau_des_fltrd, Fs, fc_cur_Hz, 'Desired torque \tau_{des} — PSD (raw vs filtered)');
% % end
% % % ==============================================================

% % ---------------- local plotting helpers ----------------
% function plotMatrixTime(t, raw, flt, ttl, ylab)
%     figure('Color','w','Name',ttl); tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
%     n = min(6, size(raw,2));
%     for jj = 1:n
%         nexttile;
%         plot(t, raw(:,jj), 'k-', 'LineWidth', 0.7); hold on;
%         plot(t, flt(:,jj), 'r-', 'LineWidth', 1.2);
%         grid on; title(sprintf('%s — Joint %d', ttl, jj), 'Interpreter','none');
%         xlabel('Time (s)'); ylabel(ylab);
%         legend('raw','filtered','Location','best');
%     end
% end

% function plotMatrixPSD(raw, flt, Fs, fcHz, ttl)
%     figure('Color','w','Name',ttl); tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
%     n = min(6, size(raw,2));
%     for jj = 1:n
%         nexttile;
%         [Praw,f] = pwelch(raw(:,jj), [], [], [], Fs);
%         [Pflt,~] = pwelch(flt(:,jj), [], [], [], Fs);
%         semilogy(f, Praw, 'k-', 'LineWidth', 0.7); hold on;
%         semilogy(f, Pflt, 'r-', 'LineWidth', 1.2);
%         xline(fcHz, 'k--', sprintf('fc = %.3f Hz', fcHz), 'LabelOrientation','horizontal');
%         grid on; xlabel('Frequency (Hz)'); ylabel('PSD');
%         title(sprintf('%s — Joint %d', ttl, jj), 'Interpreter','none');
%         legend('raw','filtered','Location','best');
%     end
% end

% end
