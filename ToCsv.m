% 假設你已經有以下變數：
% pi_lgr_base: 40x1 symbolic variable (e.g., i6_yz)
% sol.pi_b: 40x1 double, 對應的值

% Step 1: 取得 symbolic 名稱（轉為字串）
base_param_names = arrayfun(@char, pi_lgr_base, 'UniformOutput', false);

% Step 2: 將名稱與數值配對成 table
T = table(base_param_names(:), sol.pi_b(:), ...
          'VariableNames', {'Symbol', 'Value'});

% Step 3: 輸出成 CSV
writetable(T, 'base_parameters.csv');

disp('✅ base_parameters.csv 已儲存完成');
