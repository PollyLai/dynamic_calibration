function calculate_and_save_dynamics(path_to_data, sol, output_csv_path)
    % 讀取 CSV 文件
    data = parseURData(path_to_data, 1, 1400);

    % 提取關節位置、速度、加速度等
    % t = data.sim_time_s;             % 時間
    % q = data{:, {'q1', 'q2', 'q3', 'q4', 'q5', 'q6'}};  % 關節位置
    % qd = data{:, {'qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6'}};  % 關節速度
   
    t = data.t;             % 時間
    q = data.q;  % 關節位置
    qd = data.qd;  % 關節速度
    
    % pi_s = [];
    % % 假設你已經有 sol.pi_s 作為標準參數，並且已經載入
    % for i = 1:6
    %     % 每個連桿的 10 個參數
    %     link_params = sol.pi_s((i-1)*10+i:i*11-1);  % 提取前 10 個參數
    % 
    %     % 將每個連桿的參數拼接到 pi_s 中
    %     pi_s = [pi_s; link_params'];
    % end
    pi_s = sol.pi_s;
    % 預設 M, C, G 儲存容器
    M_all = cell(length(t), 1);
    C_all = cell(length(t), 1);
    G_all = cell(length(t), 1);

    % 計算每一時間步的 M, C, G
    for i = 1:length(t)
        % 取出當前時間步的 q, qd, qdd
        q_current = q(i, :)';
        qd_current = qd(i, :)';
        
        % 計算 M, C, G
        M_matrix = M_mtrx_fcn(q_current, pi_s);    % 計算 M
        C_matrix = C_mtrx_fcn(q_current, qd_current, pi_s);    % 計算 C
        G_vector = G_vctr_fcn(q_current, pi_s);   % 計算 G

        % 保存計算結果
        M_all{i} = M_matrix;
        C_all{i} = C_matrix;
        G_all{i} = G_vector;
    end

    % 將 M, C, G 存儲為 .csv 文件
    M_combined = cell2mat(M_all);  % 將 M 存為矩陣
    C_combined = cell2mat(C_all);  % 將 C 存為矩陣
    G_combined = cell2mat(G_all);  % 將 G 存為矩陣

    % 儲存為 CSV 檔案
    writetable(array2table(M_combined), fullfile(output_csv_path, 'M_matrix.csv'));
    writetable(array2table(C_combined), fullfile(output_csv_path, 'C_matrix.csv'));
    writetable(array2table(G_combined), fullfile(output_csv_path, 'G_vector.csv'));

    disp('M, C, and G matrices have been calculated and saved to CSV.');
end
