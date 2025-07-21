function generate_friction_eq(n_links)

% Create symbolic generilized coordiates, their first and second deriatives
qd_sym = sym('qd%d',[n_links,1],'real');

% Create symbolic experssions for friction parameters
pi_frcn = sym('pi_frcn_%d%d', [n_links*3,1], 'real');
arrayfun(@char, qd_sym, 'UniformOutput', false)
arrayfun(@char, pi_frcn, 'UniformOutput', false)


% Friction torque
pi_frcn_tmp = reshape(pi_frcn, [3, n_links])';
tau_frcn = pi_frcn_tmp(:,1).*qd_sym + pi_frcn_tmp(:,2).*sign(qd_sym) + pi_frcn_tmp(:,3);


% Generate a fucnction from symbolic expressions
matlabFunction(tau_frcn, 'File','autogen/F_vctr_fcn',...
               'Vars',{qd_sym, pi_frcn}, 'Optimize', true);