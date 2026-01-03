function [zmp_x, zmp_y] = compute_zmp(q, qd, q2d, ur10)
    % 這是簡化版的 ZMP 計算 (基於所有連桿的質心)
    % g = 9.81;
    % 實際上建議使用 RNEA 或是工具箱提供的總受力函數
    
    % 1. 獲取底座（Ankle）受到的總力矩 M 和總力 F
    % 這裡假設你的機器人庫有類似 'inverse_dynamics' 的功能，
    % 且能返回底座與地面接觸點的 Wrench (F, M)
    [tau, wrench_base] = inverse_dynamics(ur10, q, qd, q2d); 
    
    % wrench_base 通常包含 [Fx, Fy, Fz, Mx, My, Mz]
    Fz = wrench_base(3);
    Mx = wrench_base(4);
    My = wrench_base(5);
    
    % 2. 計算 ZMP (假設腳底板在 z=0 平面)
    % 公式：x_zmp = -My / Fz,  y_zmp = Mx / Fz
    % 注意：符號取決於你的座標系定義
    zmp_x = -My / Fz;
    zmp_y =  Mx / Fz;
end