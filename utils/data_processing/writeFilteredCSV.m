function writeFilteredCSV(data, filename)
% 將 filterData() 的輸出 data 寫成 CSV
% 欄位對應：
%   t -> sim_time_s
%   q -> q1..q6
%   qd_fltrd -> qd1..qd6
%   q2d_est -> q2d1..q2d6
%   i 或 i_fltrd -> tau1..tau6

    if nargin < 2, filename = 'filtered_data.csv'; end

    % --- 取資料 ---
    t   = data.t(:);             % Nx1
    Q   = data.q;                % NxD
    QD  = data.qd_fltrd;         % NxD
    Q2D = data.q2d_est;          % NxD
    if isfield(data, 'i_fltrd'), TAU = data.i; else, TAU = data.i; end

    N = numel(t);
    D = size(Q,2);

    % --- 基本檢查（避免用 [N 1] 這種比法）---
    assert(size(Q,1)   == N, 'q 長度與 t 不一致');
    assert(size(QD,1)  == N, 'qd_fltrd 長度與 t 不一致');
    assert(size(Q2D,1) == N, 'q2d_est 長度與 t 不一致');
    assert(size(TAU,1) == N, 'i/i_fltrd 長度與 t 不一致');

    assert(size(QD,2)  == D, 'qd_fltrd 的關節數與 q 不一致');
    assert(size(Q2D,2) == D, 'q2d_est 的關節數與 q 不一致');
    assert(size(TAU,2) == D, 'i/i_fltrd 的關節數與 q 不一致');
    % --- 組表頭 ---
    q_names   = arrayfun(@(k)sprintf('q%d',k),   1:6, 'uni', 0);
    qd_names  = arrayfun(@(k)sprintf('qd%d',k),  1:6, 'uni', 0);
    q2d_names = arrayfun(@(k)sprintf('qdd%d',k), 1:6, 'uni', 0);
    tau_names = arrayfun(@(k)sprintf('tau%d',k), 1:6, 'uni', 0);

    % --- 組 Table ---
    T = table(t, 'VariableNames', {'sim_time_s'});
    T = [T array2table(Q,   'VariableNames', q_names)];
    T = [T array2table(QD,  'VariableNames', qd_names)];
    T = [T array2table(Q2D, 'VariableNames', q2d_names)];
    T = [T array2table(TAU, 'VariableNames', tau_names)];

    % --- 輸出 ---
    writetable(T, filename);
    fprintf('CSV 已輸出：%s\n', filename);
end