function T = test_base_params(path_to_urdf, n_links, baseQR, pi_b_est, csv_path)
% 將 URDF 的標準參數映到 base 參數，並與鑑別結果比較
%
% path_to_urdf : 機械手臂 URDF 路徑
% n_links      : 關節/連桿數（例如 6）
% baseQR       : 結構，含 fields:
%                - permutationMatrix (E)
%                - beta
%                - numberOfBaseParameters (bb)
% pi_b_est     : 你鑑別得到的 base 參數（列向量，長度=bb）
% csv_path     : （可選）輸出 CSV 的路徑

    % 1) 解析 URDF，取得標準參數
    robot = parse_urdf(path_to_urdf, n_links);
    % 需有欄位：I_vec(6xn), h(3xn), m(1xn)。若 parse_urdf 已幫你填好這些就 OK
    assert(isfield(robot,'I_vec') && isfield(robot,'h') && isfield(robot,'m'), ...
        'parse_urdf must provide fields: I_vec (6xn), h (3xn), m (1xn).');

    % ***重要***：確認 baseQR 是「含馬達慣量」或「不含馬達慣量」
    % 下方示範「含馬達慣量」情形：每 link 11 個 [Ivec(6), h(3), m, im]
    % 若 baseQR 是不含馬達版（每 link 10 個），請把 im 相關行移除與維度改成 10。
    im = zeros(1, n_links);   % 如果沒有實際的 im，可先設 0（或填入你的名目值）

    pi_std = zeros(n_links*11, 1); % 若是 10 參數版改為 zeros(n_links*10,1)
    for k = 1:n_links
        base = (k-1)*11; % 若 10 參數版，改成 *10
        pi_std(base+1:base+6) = robot.I_vec(:,k); % [Ixx Ixy Ixz Iyy Iyz Izz]
        pi_std(base+7:base+9) = robot.h(:,k);     % [hx hy hz] = m*c
        pi_std(base+10)       = robot.m(k);       % m
        pi_std(base+11)       = im(k);            % im（不含馬達版請移除此行）
    end

    % 2) 用 baseQR 做映射：pi_b_urdf = [I, beta] * (E' * pi_std)
    E  = baseQR.permutationMatrix;
    B  = baseQR.beta;
    bb = baseQR.numberOfBaseParameters;

    assert(size(E,1) == numel(pi_std), ...
        'E has %d rows but pi_std has %d elements; check parameter layout.', size(E,1), numel(pi_std));
    assert(numel(pi_b_est) == bb, 'Length mismatch: pi_b_est (%d) vs bb (%d).', numel(pi_b_est), bb);

    pi_b_urdf = [eye(bb), B] * (E' * pi_std);

    % 3) 誤差分析
    abs_err = pi_b_urdf - pi_b_est(:);
    rel_err_pct = 100 * abs_err ./ max(1e-12, abs(pi_b_est(:)));

    % 4) 組表格、（可選）輸出 CSV
    Name = arrayfun(@(i) sprintf('pi_b_%d', i), 1:bb, 'UniformOutput', false).';
    T = table(Name, pi_b_urdf, pi_b_est(:), abs_err, rel_err_pct, ...
        'VariableNames', {'Parameter','FromURDF','Estimated','AbsError','RelErrorPct'});

    if nargin >= 5 && ~isempty(csv_path)
        writetable(T, csv_path);
        fprintf('Saved comparison to %s\n', csv_path);
    end
end
