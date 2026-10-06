function R = refine_design(P,kind,progress)
% Ограниченное уточнение СК по полной модели. Эталон не используется.
% progress(info) возвращает true для досрочной остановки пользователем.
if nargin<3, progress=[]; end
[initial,keys]=ACS.gain_settings(P,kind);
allkeys={'Kn','Kw','Ki','Ktheta'};
indices=cellfun(@(s)find(strcmp(allkeys,s)),keys);
lo=P.refinement.lower(indices); hi=P.refinement.upper(indices);
x0=cellfun(@(s)initial.(s),keys);
assert(all(x0>lo & x0<hi),'Начальная настройка должна лежать внутри ограничений.');
clock=tic; evaluations=0; iterations=0; cancelled=false; reached=false;
best=initial; bestJ=Inf; history=zeros(0,4); lastUpdate=0;
M0=ACS.reference_metrics(P,kind,initial,P.refinement.validation_dt);
% Сохраняем форму начального стандартного полинома с добавленным приводом.
target=P.design.result.(kind).target;
if P.actuator.dynamic
    target=conv(target,[1 2*P.actuator.zeta*P.actuator.Omega P.actuator.Omega^2]);
end
target=target/target(1);
dcFloor=0;
if strcmp(kind,'static_ny')
    [p0,n0]=ACS.characteristic(P,kind,initial);
    dcFloor=max(0,n0(end)/p0(end));
end
objective(encode(x0));
% Несколько стартов формируются только из рассчитанных СК, без использования справочных коэффициентов.
factors=[1 .55 1.5 .25];
for attempt=1:numel(factors)
    if stop_now(), break; end
    remaining=P.refinement.max_iterations-iterations;
    allowance=max(1,floor(remaining/(numel(factors)-attempt+1)));
    seed=min(hi-1e-8,max(lo+1e-8,x0*factors(attempt)));
    options=optimset('Display','off','MaxIter',allowance, ...
        'MaxFunEvals',max(1,P.refinement.max_evaluations-evaluations), ...
        'TolX',1e-5,'TolFun',1e-5,'OutputFcn',@output);
    fminsearch(@objective,encode(seed),options);
end
M=ACS.reference_metrics(P,kind,best,P.refinement.validation_dt);
if cancelled, reason='Остановлено пользователем';
elseif M.met, reason='Целевые показатели достигнуты';
elseif toc(clock)>=P.refinement.max_seconds, reason='Достигнут лимит времени';
elseif evaluations>=P.refinement.max_evaluations, reason='Достигнут лимит вычислений';
elseif iterations>=P.refinement.max_iterations, reason='Достигнут лимит итераций';
else, reason='Завершены ограниченные попытки поиска'; end
if ~M.stable, reason=[reason '. Устойчивая настройка не найдена']; end
R=struct('initial',initial,'refined',best,'initial_metrics',M0,'metrics',M, ...
    'iterations',iterations,'evaluations',evaluations,'seconds',toc(clock), ...
    'reason',reason,'history',history,'objective',bestJ,'target_polynomial',target, ...
    'dc_floor',dcFloor,'settings',P.refinement, ...
    'initial_hurwitz',ACS.hurwitz_test(ACS.characteristic(P,kind,initial)), ...
    'refined_hurwitz',ACS.hurwitz_test(ACS.characteristic(P,kind,best)));

    function z=encode(x)
        r=(x-lo)./(hi-lo); z=log(r./(1-r));
    end
    function g=decode(z)
        x=lo+(hi-lo)./(1+exp(-max(-40,min(40,z))));
        g=initial;
        for j=1:numel(keys), g.(keys{j})=x(j); end
    end
    function stop=stop_now()
        stop=cancelled || reached || iterations>=P.refinement.max_iterations || ...
            evaluations>=P.refinement.max_evaluations || toc(clock)>=P.refinement.max_seconds;
    end
    function J=objective(z)
        if stop_now(), J=1e12; return; end
        evaluations=evaluations+1;
        g=decode(z); [p,~]=ACS.characteristic(P,kind,g);
        H=ACS.hurwitz_test(p);
        if ~H.stable
            % Корни дают направление штрафа, вердикт даёт критерий Гурвица.
            J=1e8+1e5*max(0,max(real(roots(p))));
            return
        end
        Mtest=ACS.reference_metrics(P,kind,g);
        Ts=Mtest.settling;
        if ~isfinite(Ts), Ts=2*P.refinement.horizon; end
        eOS=max(0,Mtest.overshoot/P.refinement.max_overshoot-1);
        eT=max(0,Ts/P.refinement.target_time-1);
        eDC=max(0,(dcFloor-Mtest.steady)/max(dcFloor,1e-8));
        eCoeff=mean(((p/p(1)-target)./max(abs(target),1)).^2);
        J=100*(eOS^2+eT^2)+1e5*eDC^2+Mtest.ise+0.01*eCoeff;
        if ~isfinite(J), J=1e10; end
        if J<bestJ && Mtest.steady>=dcFloor-1e-8
            bestJ=J; best=g;
            history(end+1,:)=[evaluations J Mtest.overshoot Mtest.settling];
            if Mtest.met
                fine=ACS.reference_metrics(P,kind,g,P.refinement.validation_dt);
                reached=fine.met;
            end
        end
    end
    function stop=output(~,~,state)
        if strcmp(state,'iter'), iterations=iterations+1; end
        if ~isempty(progress) && (toc(clock)-lastUpdate>.25 || strcmp(state,'done'))
            info=struct('iterations',iterations,'evaluations',evaluations, ...
                'seconds',toc(clock),'best',best,'objective',bestJ);
            cancelled=logical(progress(info)); lastUpdate=toc(clock);
        end
        stop=stop_now();
    end
end
