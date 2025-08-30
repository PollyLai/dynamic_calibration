c_pol = getPolCoeffs(traj_par.T, a, b, traj_par.wf, traj_par.N, traj_par.q0);
time_step= [0:0.01:traj_par.T]
[q,qd,q2d] = mixed_traj(time_step, c_pol, a, b, traj_par.wf, traj_par.N);

figure
writetable(array2table([time_step;q;qd;q2d]', 'VariableNames', ["time","q1","q2","q3","q4","q5","q6","q1d","q2d","q3d","q4d","q5d","q6d","q1d2","q2d2","q3d2","q4d2","q5d2","q6d2"]), 'q.csv')
subplot(3,1,1)
    plot(time_step,q)
    ylabel('$q$','interpreter','latex')
    grid on
    legend('q1','q2','q3','q4','q5','q6')
subplot(3,1,2)
    plot(time_step,qd)
    ylabel('$\dot{q}$','interpreter','latex')
    grid on
    legend('qd1','qd2','qd3','qd4','qd5','qd6')
subplot(3,1,3)
    plot(time_step,q2d)
    ylabel('$\ddot{q}$','interpreter','latex')
    grid on
    legend('q2d1','q2d2','q2d3','q2d4','q2d5','q2d6')