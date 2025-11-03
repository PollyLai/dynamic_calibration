function param_sets = create_param_sets_frictionless(varargin)

    n_sets = nargin / 3;  
    param_sets(n_sets) = struct();
    
    for i = 1:n_sets
        idx = (i-1)*3 + 1;
        param_sets(i).pi_b = varargin{idx};
        param_sets(i).drvGains = varargin{idx + 1};
        param_sets(i).label = varargin{idx + 2};
    end
end
