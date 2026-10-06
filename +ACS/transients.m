function T = transients(P,kind,g,tend)
% Три независимых единичных воздействия, порядок задание / момент / поток.
T.t=(0:P.dt:tend).'; T.y=nan(numel(T.t),3); T.steady=nan(1,3);
T.stable=ACS.hurwitz_test(ACS.characteristic(P,kind,g)).stable;
if ~T.stable, return; end
Q=ACS.apply_settings(P,kind,g); sys=ACS.closed_loop(Q,kind);
out=1; amplitude=P.command.ny;
if endsWith(kind,'pitch'), out=3; amplitude=P.command.theta; end
inputs=[1 3 2]; amps=[amplitude P.command.moment P.command.gust];
for j=1:3
    % Удаляем свободный theta-интегратор из наблюдаемого канала ny.
    s=minreal(sys(out,inputs(j)),1e-9,false);
    T.y(:,j)=step(s,T.t)*amps(j); T.steady(j)=dcgain(s)*amps(j);
end
end
