
function Yi = standard_regressor_UR10E(q,qd,q2d)
    % persistent Y_handle
    % if isempty(Y_handle)
    %     Y_handle = load('autogen/standard_regressor_UR10E.mat', ...
    %                     'Y_handle').Y_handle;
    %     fprintf("Check standare regresso\n ");

    % end
    % Yi = Y_handle(q,qd,q2d);

    persistent f
    if isempty(f)
        tmp = load('autogen/standard_regressor_UR10E.mat','eval_handle');
        f   = tmp.eval_handle;            % 只載入一次
    end
    Yi = f(q,qd,q2d);                     % 之後快速呼叫
end