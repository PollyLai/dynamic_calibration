function [tbl_flat, per_link] = pi_s_report(sol)
% Create readable report from sol.pi_s (66x1 standard parameters).
% Outputs:
%   tbl_flat : a flat table of all 66 parameters + derived COM (cx,cy,cz per link)
%   per_link : struct(1x6) with fields I_body(3x3), m, c(3x1), Irot

    assert(isfield(sol,'pi_s') && numel(sol.pi_s)==66, ...
        'sol.pi_s must be 66x1 (11 per link).');

    pi = sol.pi_s(:);
    L = 6;
    names = ["Ixx","Ixy","Ixz","Iyy","Iyz","Izz","hx","hy","hz","m","Irot"];
    rows = [];
    per_link = struct('I_body',cell(1,L),'m',[],'c',[],'Irot',[]);
    for k = 1:L
        b = (k-1)*11;
        Ixx = pi(b+1); Ixy = pi(b+2); Ixz = pi(b+3);
        Iyy = pi(b+4); Iyz = pi(b+5); Izz = pi(b+6);
        hx  = pi(b+7); hy  = pi(b+8); hz  = pi(b+9);
        m   = pi(b+10); Irot = pi(b+11);

        I = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
        if m>0, c = [hx;hy;hz]/m; else, c = [NaN;NaN;NaN]; end

        per_link(k).I_body = I;
        per_link(k).m = m;
        per_link(k).c = c;
        per_link(k).Irot = Irot;

        vals = [Ixx Ixy Ixz Iyy Iyz Izz hx hy hz m Irot];
        for j = 1:numel(names)
            rows = [rows; {k, char(names(j)), vals(j)}]; %#ok<AGROW>
        end
        % also append derived COM
        rows = [rows; {k,'cx',c(1)}; {k,'cy',c(2)}; {k,'cz',c(3)}];
    end
    tbl_flat = cell2table(rows, 'VariableNames', {'link','name','value'});
end
