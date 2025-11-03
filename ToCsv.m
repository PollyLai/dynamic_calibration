% % 假設你已經有以下變數：
% % pi_lgr_base: 40x1 symbolic variable (e.g., i6_yz)
% % sol.pi_b: 40x1 double, 對應的值
% 
% % Step 1: 取得 symbolic 名稱（轉為字串）
% % load('C:\Model_Identification\dynamic_calibration\combined_table.mat');
% S = load('C:\Model_Identification\dynamic_calibration\combined_table.mat');
% combined_table = S.combined_table;  % 取出 table
% 
% % 若第 1 欄是 symbolic，先轉成字串
% firstVar = combined_table.Properties.VariableNames{1};
% if isa(combined_table.(firstVar), 'sym')
%     combined_table.(firstVar) = cellstr(arrayfun(@char, combined_table.(firstVar), 'UniformOutput', false));
% else
%     % 確保是文字型別（以防萬一是 categorical / string）
%     combined_table.(firstVar) = string(combined_table.(firstVar));
% end
% 
% % 可選：統一欄位名稱（依你截圖）
% if width(combined_table) >= 4
%     combined_table.Properties.VariableNames(1:4) = {'vars','isaac','base','dynamic'};
% end
% 
% writetable(combined_table, 'combined_table.csv');  % 直接輸出
% disp('done');




base_param_names = arrayfun(@char, pi_lgr_base, 'UniformOutput', false);

% Step 2: 將名稱與數值配對成 table
T = table(base_param_names(:), sol.pi_b(:), ...
          'VariableNames', {'Symbol', 'Value'});

% Step 3: 輸出成 CSV
writetable(T, 'base_parameters.csv');

disp('base_parameters.csv is done');
