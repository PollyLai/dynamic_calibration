function Yi = standard_regressor_UR10E(q,qd,q2d)
    persistent Y_handle
    if isempty(Y_handle)
        Y_handle = load('autogen/standard_regressor_UR10E.mat', ...
                        'Y_handle').Y_handle;
    end
    Yi = Y_handle(q,qd,q2d);
end