%% 0. 環境準備與模型載入 (包含所有錯誤修正)
close all;
clc;

% PATH: 1. urdf; 2. telemetry; 3. outdir;  
path_to_urdf = 'test_inverted_final_6.urdf';
try
    robot = importrobot(path_to_urdf);
catch ME
    error('無法載入 URDF 檔案。請確認檔案路徑是否正確。');
end
robot.Gravity = [0, 0, -9.81]; % [gx, gy, gz] 
robot.DataFormat = 'row'; 


Q_table = readtable("ur10_simulation_telemetry_1222.csv"); % in radians
numJoints = 6;
COMPUTE_FULL_C = true;
outdir = 'output_csv';

Q = Q_table{:, {'q1', 'q2', 'q3', 'q4', 'q5', 'q6'}}; % 提取 q 數據
Qd = Q_table{:, {'qd1','qd2','qd3','qd4','qd5','qd6'}};
N = size(Q, 1);



% --- Prealloc ---
M_urdf_diag = zeros(N, numJoints);
g_urdf      = zeros(N, numJoints);
Cqdot_urdf  = zeros(N, numJoints);
M_urdf_full = zeros(N, numJoints * numJoints);
if COMPUTE_FULL_C
    C_urdf_full = zeros(N, numJoints * numJoints);
end

qd_zero  = zeros(1, numJoints);
qdd_zero = zeros(1, numJoints);

% --- Helper for full C (numerical, per-column with unit velocity) ---
function Cmat = compute_C_matrix(robot, q, numJoints)
    g_here = inverseDynamics(robot, q, zeros(1, numJoints), zeros(1, numJoints)); % g(q)
    Cmat = zeros(numJoints);
    for j = 1:numJoints
        ej = zeros(1, numJoints);
        ej(j) = 1.0;
        nle_j = inverseDynamics(robot, q, ej, zeros(1, numJoints)); % = C*e_j + g
        Cmat(:, j) = (nle_j - g_here).';
    end
end

for i = 1:N
    q = Q(i, :);
    qd = Qd(i, :);
    % 1) Mass matrix M(q)
    M_mat = massMatrix(robot, q);          % 6x6
    M_urdf_diag(i, :) = diag(M_mat).';     % store diagonal
    M_urdf_full(i, :) = reshape(M_mat.', 1, []); % row-flatten (col-major in MATLAB => transpose for row-major-like)

    % 2) Gravity g(q) = inverseDynamics(q, 0, 0)
    g_vec = inverseDynamics(robot, q, qd_zero, qdd_zero);
    g_urdf(i, :) = g_vec.';

    % 3) C(q,qd)*qd = inverseDynamics(q, qd, 0) - g(q)
    tau_cg = inverseDynamics(robot, q, qd, qdd_zero);
    Cqdot  = tau_cg - g_vec;
    Cqdot_urdf(i, :) = Cqdot.';

    % 4) (optional) Full C matrix
    if COMPUTE_FULL_C
        Cmat = compute_C_matrix(robot, q, numJoints);
        C_urdf_full(i, :) = reshape(Cmat.', 1, []);
    end
    M_mat = massMatrix(robot, q);
    M_urdf_diag(i, :) = diag(M_mat)';

    % 重力向量 g (設定速度和加速度為零)
    g_vec = inverseDynamics(robot, q, qd_zero, qdd_zero);
    g_urdf(i, :) = g_vec';
end
Tg = array2table(g_urdf, 'VariableNames', {'g1','g2','g3','g4','g5','g6'});
writetable(Tg, fullfile(outdir, 'g.csv'));

M_labels = strings(1, numJoints*numJoints);
k = 1;
for r = 1:numJoints
    for c = 1:numJoints
        M_labels(k) = sprintf('M[%d,%d]', r, c);
        k = k + 1;
    end
end
TM = array2table(M_urdf_full, 'VariableNames', cellstr(M_labels));
writetable(TM, fullfile(outdir, 'M_full.csv'));
TCq = array2table(Cqdot_urdf, 'VariableNames', {'Cqdot1','Cqdot2','Cqdot3','Cqdot4','Cqdot5','Cqdot6'});
writetable(TCq, fullfile(outdir, 'Cqdot.csv'));

% optional full C
if COMPUTE_FULL_C
    C_labels = strings(1, numJoints*numJoints);
    k = 1;
    for r = 1:numJoints
        for c = 1:numJoints
            C_labels(k) = sprintf('C[%d,%d]', r, c);
            k = k + 1;
        end
    end
    TC = array2table(C_urdf_full, 'VariableNames', cellstr(C_labels));
    writetable(TC, fullfile(outdir, 'C_full.csv'));
end


assert(exist('M_mtrx_fcn', 'file')==2, '找不到 M_mtrx_fcn.m');
assert(exist('G_vctr_fcn', 'file')==2, '找不到 G_vctr_fcn.m');
hasC = (exist('C_mtrx_fcn', 'file')==2);  %#ok<NASGU> % 目前不畫 C，但可計算以供檢查

robot_parsed = parse_urdf(path_to_urdf, dof);   % 你提供的 parser（會組好每節的 10 維 π_i）

% 組成整條手臂的 60×1 參數向量 pi = [π1; π2; …; π6]
% 其中 π_i = [Ixx+m(y^2+z^2); Ixy-mxy; Ixz-mxz; Iyy+m(x^2+z^2); Iyz-myz; Izz+m(x^2+y^2); mx; my; mz; m]
pi_vec = reshape(robot_parsed.pi, [], 1);  % (10×6) → (60×1)，按 link1..link6 堆疊

% 沿軌跡用 autogen 函式計算
M_calc_diag = zeros(N, numJoints);
g_calc      = zeros(N, numJoints);
for i = 1:N
    q  = Q(i, :);

    % M（做數值對稱化避免微小非對稱）
    M_i = M_mtrx_fcn(q.', pi_vec);
    M_i = 0.5 * (M_i + M_i.');
    M_calc_diag(i, :) = diag(M_i).';

    % g
    g_calc(i, :) = G_vctr_fcn(q.', pi_vec).';
end

%% 6) 畫圖：M 對角（x: MATLAB toolbox, y: calc）
figure('Color','w','Position',[100 100 1100 620]);
tiledlayout(2,3,'Padding','compact','TileSpacing','compact');
for j = 1:numJoints
    nexttile;
    x = M_urdf_diag(:, j);   % MATLAB toolbox
    y = M_calc_diag(:, j);   % your generated function
    scatter(x, y, 12, 'filled'); hold on; grid on;
    % 45 度參考線
    lo = min([x; y]); hi = max([x; y]);
    pad = 0.05*max(hi-lo, 1);
    lo = lo - pad; hi = hi + pad;
    plot([lo hi],[lo hi],'k--'); xlim([lo hi]); ylim([lo hi]);
    % 皮爾森 r
    if std(x)>0 && std(y)>0, r = corr(x, y); else, r = NaN; end
    title(sprintf('M%d%d  (r=%.3f)', j, j, r));
    xlabel('MATLAB Toolbox (kg·m^2)');
    ylabel('Calculated from fcn (kg·m^2)');
end
sgtitle('Mass Matrix Diagonal: MATLAB vs. Calculated-from-fcn');
saveas(gcf, fullfile(outdir, 'fig_Mdiag_matlab_vs_calc.png'));

%% 7) 畫圖：g（x: MATLAB toolbox, y: calc）
figure('Color','w','Position',[100 100 1100 620]);
tiledlayout(2,3,'Padding','compact','TileSpacing','compact');
for j = 1:numJoints
    nexttile;
    x = g_urdf(:, j);      % MATLAB toolbox
    y = g_calc(:, j);      % your generated function
    scatter(x, y, 12, 'filled'); hold on; grid on;
    lo = min([x; y]); hi = max([x; y]);
    pad = 0.05*max(hi-lo, 1);
    lo = lo - pad; hi = hi + pad;
    plot([lo hi],[lo hi],'k--'); xlim([lo hi]); ylim([lo hi]);
    if std(x)>0 && std(y)>0, r = corr(x, y); else, r = NaN; end
    title(sprintf('g_%d  (r=%.3f)', j, r));
    xlabel('MATLAB Toolbox Torque (N·m)');
    ylabel('Calculated from fcn Torque (N·m)');
end
sgtitle('Gravity Vector: MATLAB vs. Calculated-from-fcn');
saveas(gcf, fullfile(outdir, 'fig_g_matlab_vs_calc.png'));