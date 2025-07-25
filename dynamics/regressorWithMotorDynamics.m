function Y = regressorWithMotorDynamics(q,qd,q2d, n_links)
% ----------------------------------------------------------------------
% This function adds motor dynamics to rigid body regressor.
% It is simplified model of motor dynamics, it adds only reflected
% inertia i.e. I_rflctd = Im*N^2 where N is reduction ratio - I_rflctd*q_2d
% parameter is added to existing vector of each link [pi_i I_rflctd_i]
% so that each link has 11 parameters
% ----------------------------------------------------------------------
assert(iscolumn(q)  && iscolumn(qd) && iscolumn(q2d), ...
        'q, qd, q2d are not nx1 vectors');
assert(((numel(q)==n_links) && (numel(qd)==n_links) && (numel(q2d)==n_links)), ...
    'vector lengths q, qd, q2d are not correspond with n_links');

Y_rgd_bdy = standard_regressor_UR10E(q, qd, q2d);      % n × 10n
Y_mtr = diag(q2d); 
Y = zeros(n_links, 11*n_links, 'like', Y_rgd_bdy);
    for k = 1:n_links
        Y(:, (k-1)*11 + (1:11)) = [ ...
            Y_rgd_bdy(:, (k-1)*10 + (1:10)), ...  % 10 rigid cols
            Y_mtr(:, k)                       % 1 motor col
        ];
    end
end
% if size(q,1)==6 && size(q,2)==1 && size(qd,1)==6 && size(qd,2)==1 ...
%         && size(q2d,1)==6 && size(q2d,2)==1
%     Y_rgd_bdy = standard_regressor_UR10E(q,qd,q2d);
%     Y_mtrs = diag(q2d);
%     Y = [Y_rgd_bdy(:,1:10), Y_mtrs(:,1), Y_rgd_bdy(:,11:20), Y_mtrs(:,2),...
%          Y_rgd_bdy(:,21:30), Y_mtrs(:,3), Y_rgd_bdy(:,31:40), Y_mtrs(:,4),...
%          Y_rgd_bdy(:,41:50), Y_mtrs(:,5), Y_rgd_bdy(:,51:60), Y_mtrs(:,6)];
% else
%     error('Input dimension error!')
% end


