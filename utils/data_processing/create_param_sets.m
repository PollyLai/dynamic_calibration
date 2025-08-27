function param_sets = create_param_sets(varargin)

    n_sets = nargin / 4;
    param_sets(n_sets) = struct();
    
    for i = 1:n_sets
        idx = (i-1)*4 + 1;
        param_sets(i).pi_b = varargin{idx};
        param_sets(i).pi_fr = varargin{idx + 1};
        param_sets(i).drvGains = varargin{idx + 2};
        param_sets(i).label = varargin{idx + 3};
    end
end