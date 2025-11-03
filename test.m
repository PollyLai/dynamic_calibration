% sol = estimate_dynamic_params(path_to_est_data, idxs, ...
%                               drive_gains, baseQR, 'PC-OLS');
% rre = validate_dynamic_params(path_to_val_data, idxs, ...
%                               drive_gains, baseQR, sol.pi_b, sol.pi_fr)
% q, qd, qdd 皆為 6×3；第 k 欄是第 k 個測試樣本
% in1: q (rad), in2: qd (rad/s), in3: qdd (rad/s^2)
% Columns: A=max||q||, B=max||qd||, C=max||qdd||

%% test regressor
% in1 = [ 3.14348245,   1.20708394,   0.000598620507;
%        -1.24314737,  -1.92887735,  -1.57081258;
%        -1.43700123,  -1.24068928,  -0.00140677684;
%        -1.02564430,  -0.790616989, -1.59533346;
%         1.45147157,   1.05687165,   0.00102807931;
%         0.267750055,  1.56831479,  -0.00116238242 ];
% 
% in2 = [ -0.0615001395, -1.77138686,   0.0425403006;
%          0.342190057,  -0.925295055, -0.0390159264;
%         -0.0403537452, -0.00205661799, -0.134035826;
%          0.249705195,  -0.0271375645, -0.707439423;
%          0.347629100,  -0.645045996,   0.0888119936;
%          1.27705324,    0.0579800084, -0.096743755 ];
% 
% in3 = [ -1.41644201,    0.0575095388,  -0.184738814;
%         -0.865724642,  -0.143635266,  -1.83015159;
%          0.600614417,  -0.200410015,  -2.11700862;
%         -0.136876100,  -0.277929088,   57.707979;
%         -0.496050692,   0.0220477571,   3.81226223;
%         -0.074911114,  -0.782227386,   -3.62454375 ];
% 
% % Example usage:
% % Y_A = standard_regressor_UR10E(in1(:,1), in2(:,1), in3(:,1));
% % Y_B = standard_regressor_UR10E(in1(:,2), in2(:,2), in3(:,2));
% % Y_C = standard_regressor_UR10E(in1(:,3), in2(:,3), in3(:,3));
% 
% 
% 
% Y1_m = standard_regressor_UR10E(in1(:,1), in2(:,1), in3(:,1));
% Y2_m = standard_regressor_UR10E(in1(:,2), in2(:,2), in3(:,2));
% Y3_m = standard_regressor_UR10E(in1(:,3), in2(:,3), in3(:,3));
% 
% Y1_py = readmatrix('C:\myIsaac\repos\isaac_ldsc_bipedal\scripts\dynamic\Y1_py.csv');
% Y2_py = readmatrix('C:\myIsaac\repos\isaac_ldsc_bipedal\scripts\dynamic\Y2_py.csv');
% Y3_py = readmatrix('C:\myIsaac\repos\isaac_ldsc_bipedal\scripts\dynamic\Y3_py.csv');
% 
% max_err1 = max(abs(Y1_m(:)-Y1_py(:)))
% max_err2 = max(abs(Y2_m(:)-Y2_py(:)))
% max_err3 = max(abs(Y3_m(:)-Y3_py(:)))
% 
% % 相對誤差 (分母用 max(1, |值|) 避免除以零)
% rel_err1 = max(abs(Y1_m(:)-Y1_py(:)) ./ max(1, abs(Y1_m(:))))
% rel_err2 = max(abs(Y2_m(:)-Y2_py(:)) ./ max(1, abs(Y2_m(:))))
% rel_err3 = max(abs(Y3_m(:)-Y3_py(:)) ./ max(1, abs(Y3_m(:))))
% 
% fprintf('Y1: abs=%.3e, rel=%.3e\n', max_err1, rel_err1);
% fprintf('Y2: abs=%.3e, rel=%.3e\n', max_err2, rel_err2);
% fprintf('Y3: abs=%.3e, rel=%.3e\n', max_err3, rel_err3);


%% output_filtered data
path_to_data = 'ur10_simulation_telemetry_perfect_titleless.csv';     idxs = [1, 1249];
vldtnTrjctry = parseURData(path_to_data, idxs(1), idxs(2));
vldtnTrjctry = filterData(vldtnTrjctry);

writeFilteredCSV(vldtnTrjctry, 'output_csv\filterd_data.csv');