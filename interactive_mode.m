function fig = interactive_mode(visible,auto_refine)
% Одна выбранная система, ограниченный синтез, сравнение и устойчивость.
if nargin<1, visible='on'; end
if nargin<2, auto_refine=true; end
root=fileparts(mfilename('fullpath')); addpath(root);
P=ACS.default_config(); kinds={'static_ny','astatic_ny','static_pitch','astatic_pitch'};
names={'Перегрузка, статическая','Перегрузка, астатическая', ...
    'Тангаж, статический внутренний контур','Тангаж, астатический внутренний контур'};
kind=kinds{1}; keys={'Kn','Kw','Ki','Ktheta'};
saved_file=fullfile(root,'user_baselines_workflow.mat'); baselines=struct();
if isfile(saved_file)
    loaded=load(saved_file,'baselines');
    if isfield(loaded,'baselines'), baselines=loaded.baselines; end
end
cache=containers.Map(); mapCache=containers.Map(); domainCache=containers.Map(); pins={}; R=[];
initial=ACS.gain_settings(P,kind); candidate=initial;
if isfield(baselines,kind), candidate=baselines.(kind); end
busy=false; cancelled=false; closing=false; lastDrag=tic;
curveNames={}; curveGains={}; colors=[]; trace=gobjects(0,3);
mapPoints=gobjects(0); mihLines=gobjects(0); bands=gobjects(0);
tableData=cell(0,6); tEnd=10;
fig=uifigure('Name','Системы управления самолётом. Синтез и устойчивость', ...
    'Position',[25 45 1780 1000],'Visible',visible,'CloseRequestFcn',@close_window);
shell=uigridlayout(fig,[1 1]); shell.Padding=[0 0 0 0];
tabs=uitabgroup(shell,'Tag','study_tabs');
mainTab=uitab(tabs,'Title','Переходные процессы и настройка');
factorTab=uitab(tabs,'Title','Влияние c и частоты привода','Tag','factor_tab');
ensureFactors=ACS.factor_study_page(factorTab,P);
tabs.SelectionChangedFcn=@(~,e)select_page(e.NewValue);
main=uigridlayout(mainTab,[1 2]); main.ColumnWidth={330,'1x'};
left=uigridlayout(main,[23 1]); left.Layout.Column=1; left.Scrollable='on';
left.RowHeight={20,30,55,20,30,30,30,32,42,20,40,20,40,20,40,20,40,24,30,30,30,30,90};
label(left,1,'Исследуемая система');
modeBox=uidropdown(left,'Items',names,'ItemsData',kinds,'Value',kind, ...
    'Tag','mode','ValueChangedFcn',@(~,~) select_mode()); modeBox.Layout.Row=2;
pair=uigridlayout(left,[2 2]); pair.Layout.Row=3; pair.RowHeight={18,27}; pair.Padding=[0 0 0 0];
uilabel(pair,'Text','Графики, с'); uilabel(pair,'Text','Доп. сравнений');
timeBox=uidropdown(pair,'Items',{'2','4','6','8','10'},'ItemsData',2:2:10, ...
    'Value',10,'ValueChangedFcn',@(~,~) change_time());
countBox=uidropdown(pair,'Items',{'0','1','2','3','4','5'},'ItemsData',0:5,'Value',3);
label(left,4,'Частота привода, рад/с');
frequencyBox=uidropdown(left,'Items',cellstr(string(5:5:30)),'ItemsData',5:5:30, ...
    'Value',P.actuator.Omega,'Tag','frequency','ValueChangedFcn',@(~,~) change_model());
frequencyBox.Layout.Row=5;
pair=uigridlayout(left,[1 2]); pair.Layout.Row=6; pair.Padding=[0 0 0 0];
uilabel(pair,'Text','Демпфирование ξ');
dampingBox=uieditfield(pair,'numeric','Limits',[.05 2],'Value',.8, ...
    'Tag','damping','ValueChangedFcn',@(~,~) change_model());
pair=uigridlayout(left,[1 2]); pair.Layout.Row=7; pair.Padding=[0 0 0 0];
uilabel(pair,'Text','Предел итераций');
iterationBox=uispinner(pair,'Limits',[1 2000],'Step',25,'Value',200, ...
    'RoundFractionalValues','on','Tag','iterations');
pair=uigridlayout(left,[1 2]); pair.Layout.Row=8; pair.Padding=[0 0 0 0];
refineButton=uibutton(pair,'Text','Уточнить','ButtonPushedFcn',@(~,~) calculate(true));
stopButton=uibutton(pair,'Text','Остановить','Enable','off','ButtonPushedFcn',@(~,~) stop_search());
label(left,9,'Цель по заданию, σ ≤ 10%, t₅% ≤ 1 с. Расчёт показателей до 30 с.');
sliders=gobjects(1,4); captions=gobjects(1,4);
for j=1:4
    captions(j)=label(left,8+2*j,keys{j});
    sliders(j)=uislider(left,'Limits',[0 P.refinement.upper(j)],'Tag',keys{j}, ...
        'ValueChangingFcn',@(~,e) move_gain(j,e.Value,false), ...
        'ValueChangedFcn',@(~,e) move_gain(j,e.Value,true));
    sliders(j).Layout.Row=9+2*j;
end
corridorBox=uicheckbox(left,'Text','Коридор ±5% текущей кривой','Value',true, ...
    'ValueChangedFcn',@(~,~) update_current(true)); corridorBox.Layout.Row=18;
pinButton=uibutton(left,'Text','Зафиксировать для сравнения','ButtonPushedFcn',@(~,~) pin_current()); pinButton.Layout.Row=19;
baseButton=uibutton(left,'Text','Принять за исходную','ButtonPushedFcn',@(~,~) save_current()); baseButton.Layout.Row=20;
clearButton=uibutton(left,'Text','Очистить дополнительные сравнения','ButtonPushedFcn',@(~,~) clear_pins()); clearButton.Layout.Row=21;
exportButton=uibutton(left,'Text','Сохранить графики и модели','ButtonPushedFcn',@(~,~) export_results()); exportButton.Layout.Row=22;
status=label(left,23,'');
right=uigridlayout(main,[3 2]); right.Layout.Column=2;
right.ColumnWidth={'1.4x','1x'}; right.RowHeight={'1x','1x','1x'};
ax=gobjects(1,3);
for j=1:3, ax(j)=uiaxes(right); ax(j).Layout.Row=j; ax(j).Layout.Column=1; end
mapAx=uiaxes(right); mapAx.Layout.Row=1; mapAx.Layout.Column=2;
mihAx=uiaxes(right); mihAx.Layout.Row=2; mihAx.Layout.Column=2;
bottom=uigridlayout(right,[2 1]); bottom.Layout.Row=3; bottom.Layout.Column=2; bottom.RowHeight={'1x',60};
metricsTable=uitable(bottom,'ColumnName',{'Настройка','Гурвиц','σ, %','t₅%, с','y∞','Цель'}, ...
    'RowName',{},'ColumnWidth',{105,90,55,65,55,65},'Tag','metrics');
uilabel(bottom,'Text',['Синие полосы относятся только к текущей кривой. Для нулевого y∞ возмущения берётся 5% пика. ' ...
    'Граница окна не является границей устойчивости.'],'WordWrap','on');
sync_controls(); render_all();
if auto_refine, calculate(false); end

    function h=label(parent,row,txt)
        h=uilabel(parent,'Text',txt,'WordWrap','on'); h.Layout.Row=row;
    end
    function select_page(tab)
        if tab==factorTab, ensureFactors(); end
    end
    function key=cache_key()
        key=sprintf('%s_%.9g_%.9g_%d',kind,P.actuator.Omega,P.actuator.zeta,iterationBox.Value);
    end
    function select_mode()
        if busy, return; end
        kind=modeBox.Value; pins={}; R=[]; initial=ACS.gain_settings(P,kind); candidate=initial;
        if isfield(baselines,kind), candidate=baselines.(kind); end
        sync_controls(); render_all();
        if auto_refine, calculate(false); end
    end
    function change_model()
        if busy, return; end
        P.actuator.Omega=frequencyBox.Value; P.actuator.zeta=dampingBox.Value;
        pins={}; R=[]; candidate=initial;
        sync_controls(); render_all();
        if auto_refine, calculate(false); end
    end
    function change_time()
        tEnd=timeBox.Value; render_all();
    end
    function calculate(force)
        if busy, return; end
        P.refinement.max_iterations=iterationBox.Value;
        P.refinement.max_evaluations=max(100,5*iterationBox.Value);
        key=cache_key();
        if ~force && isKey(cache,key)
            R=cache(key); candidate=R.refined; restore_user(); sync_controls(); render_all();
            status.Text=R.reason; return
        end
        busy=true; cancelled=false; controls_enabled(false); status.Text='Расчёт выбранной системы'; drawnow;
        try
            R=ACS.refine_design(P,kind,@progress); cache(key)=R;
            candidate=R.refined;
            if ~force, restore_user(); end
        catch err
            busy=false;
            if isvalid(fig), controls_enabled(true); status.Text=err.message; end
            rethrow(err)
        end
        busy=false;
        if closing, delete(fig); return; end
        controls_enabled(true); sync_controls(); render_all();
        status.Text=sprintf('%s. %d итераций, %d вычислений. Уточнение σ %.2f%%, t₅%% %.3g с.', ...
            R.reason,R.iterations,R.evaluations,R.metrics.overshoot,R.metrics.settling);
    end
    function restore_user()
        if isfield(baselines,kind), candidate=baselines.(kind); end
    end
    function stop=progress(info)
        status.Text=sprintf('Итерация %d / %d. Вычислений %d. Время %.1f с.', ...
            info.iterations,P.refinement.max_iterations,info.evaluations,info.seconds);
        drawnow; stop=cancelled;
    end
    function stop_search(), cancelled=true; end
    function close_window(~,~)
        if busy, cancelled=true; closing=true; else, delete(fig); end
    end
    function controls_enabled(on)
        if on, value='on'; stop='off'; else, value='off'; stop='on'; end
        for h=[modeBox frequencyBox dampingBox iterationBox refineButton timeBox countBox ...
                pinButton baseButton clearButton exportButton corridorBox sliders]
            h.Enable=value;
        end
        stopButton.Enable=stop;
        if on, sync_controls(); end
    end
    function sync_controls()
        active=[true true startsWith(kind,'astatic_') endsWith(kind,'pitch')];
        for jj=1:4
            sliders(jj).Enable=matlab.lang.OnOffSwitchState(active(jj));
            sliders(jj).Limits=[0 max(P.refinement.upper(jj),candidate.(keys{jj})*1.1)];
            sliders(jj).Value=candidate.(keys{jj});
            captions(jj).Text=sprintf('%s = %.5g',keys{jj},candidate.(keys{jj}));
        end
    end
    function move_gain(index,value,force)
        if busy, return; end
        candidate.(keys{index})=value; captions(index).Text=sprintf('%s = %.5g',keys{index},value);
        if ~force && toc(lastDrag)<.12, return; end
        lastDrag=tic; update_current(force);
    end
    function pin_current()
        if numel(pins)>=countBox.Value, status.Text='Увеличьте число дополнительных сравнений.'; return; end
        if ~ACS.hurwitz_test(ACS.characteristic(P,kind,candidate)).stable
            status.Text='Неустойчивую настройку нельзя зафиксировать.'; return;
        end
        pins{end+1}=candidate; render_all();
    end
    function save_current()
        if ~ACS.hurwitz_test(ACS.characteristic(P,kind,candidate)).stable
            status.Text='Неустойчивую настройку нельзя сохранить как исходную.'; return;
        end
        baselines.(kind)=candidate; save(saved_file,'baselines');
        status.Text='Сохранён старт ползунков для этой системы. Расчёт СК и эталон не заменяются.';
    end
    function clear_pins(), pins={}; render_all(); end
    function text=gain_text(name,g)
        text=sprintf('%s  Kn=%.3g Kw=%.3g',name,g.Kn,g.Kw);
        if startsWith(kind,'astatic_'), text=sprintf('%s Ki=%.3g',text,g.Ki); end
        if endsWith(kind,'pitch'), text=sprintf('%s Kθ=%.3g',text,g.Ktheta); end
        text=sprintf('%s Ω=%.3g ξ=%.2g',text,P.actuator.Omega,P.actuator.zeta);
    end
    function render_all()
        curveNames={'СК, начальная'}; curveGains={initial}; colors=[0 0 0];
        if ~isempty(R)
            curveNames{end+1}='Уточнённая'; curveGains{end+1}=R.refined; colors(end+1,:)=[0 .48 .75];
        end
        curveNames{end+1}='Справочная настройка'; curveGains{end+1}=P.reference.(kind); colors(end+1,:)=[.85 .32 .05];
        extra=lines(max(1,numel(pins)));
        for jj=1:numel(pins)
            curveNames{end+1}=sprintf('Сравнение %d',jj); curveGains{end+1}=pins{jj}; colors(end+1,:)=extra(jj,:);
        end
        curveNames{end+1}='Текущая'; curveGains{end+1}=candidate; colors(end+1,:)=[.45 .15 .8];
        n=numel(curveGains); trace=gobjects(n,3); tableData=cell(n,6);
        titles={'Отработка задания','Постоянный возмущающий момент','Вход в вертикальный поток'};
        for jj=1:3
            % reset удаляет также коридоры с HandleVisibility='off'.
            cla(ax(jj),'reset'); hold(ax(jj),'on'); grid(ax(jj),'on'); xlabel(ax(jj),'t, с');
            title(ax(jj),titles{jj}); ylabel(ax(jj),'n_y, g');
            if endsWith(kind,'pitch'), ylabel(ax(jj),'Тангаж, град'); end
            xlim(ax(jj),[0 tEnd]);
        end
        bands=gobjects(0);
        for ii=1:n
            T=ACS.transients(P,kind,curveGains{ii},tEnd); style='-';
            if ii==n, style='--'; end
            for jj=1:3
                trace(ii,jj)=plot(ax(jj),T.t,T.y(:,jj),style,'Color',colors(ii,:), ...
                    'LineWidth',1.6,'DisplayName',gain_text(curveNames{ii},curveGains{ii}));
            end
            tableData(ii,:)=metric_row(curveNames{ii},curveGains{ii});
        end
        for jj=1:3, legend(ax(jj),'Location','southoutside','FontSize',8,'Interpreter','none'); end
        render_mikhailov(); update_current(true);
    end
    function row=metric_row(name,g)
        M=ACS.reference_metrics(P,kind,g,P.refinement.validation_dt);
        H=ACS.hurwitz_test(ACS.characteristic(P,kind,g)); answer='нет';
        if M.met, answer='да'; end
        row={name,H.status,round(M.overshoot,3),round(M.settling,3),round(M.steady,4),answer};
    end
    function update_current(force)
        if isempty(trace), return; end
        curveGains{end}=candidate; T=ACS.transients(P,kind,candidate,tEnd);
        for jj=1:3
            trace(end,jj).XData=T.t; trace(end,jj).YData=T.y(:,jj);
            trace(end,jj).DisplayName=gain_text('Текущая',candidate);
        end
        delete(bands(isgraphics(bands))); bands=gobjects(0);
        if corridorBox.Value && T.stable
            for jj=1:3
                dc=T.steady(jj); width=.05*abs(dc);
                if abs(dc)<1e-8, width=.05*max(abs(T.y(:,jj))); end
                if ~isfinite(width), continue; end
                bands(end+1)=patch(ax(jj),[0 tEnd tEnd 0],dc+[-width -width width width], ...
                    [.45 .65 1],'FaceAlpha',.15,'EdgeColor','none','HandleVisibility','off','Tag','corridor');
                bands(end+1)=yline(ax(jj),dc-width,':','Color',[.4 .6 .9],'HandleVisibility','off');
                bands(end+1)=yline(ax(jj),dc+width,':','Color',[.4 .6 .9],'HandleVisibility','off');
            end
        end
        tableData(end,:)=metric_row('Текущая',candidate); metricsTable.Data=tableData;
        z=ACS.mikhailov_curve(ACS.characteristic(P,kind,candidate));
        mihLines(end).XData=real(z); mihLines(end).YData=imag(z);
        mihLines(end).DisplayName=gain_text('Текущая',candidate);
        if force
            render_map();
        elseif ~isempty(mapPoints)
            mapPoints(end).XData=candidate.Kn; mapPoints(end).YData=candidate.Kw;
            title(mapAx,{'Устойчивость в плоскости Kn, Kw','Сечения обновятся после отпускания ползунка'});
        end
        setappdata(fig,'workflow',struct('P',P,'kind',kind,'result',R,'candidate',candidate,'metrics',{tableData}));
        drawnow limitrate;
    end
    function render_mikhailov()
        cla(mihAx,'reset'); hold(mihAx,'on'); grid(mihAx,'on');
        xline(mihAx,0,'HandleVisibility','off'); yline(mihAx,0,'HandleVisibility','off');
        mihLines=gobjects(1,numel(curveGains));
        for ii=1:numel(curveGains)
            z=ACS.mikhailov_curve(ACS.characteristic(P,kind,curveGains{ii})); style='-';
            if ii==numel(curveGains), style='--'; end
            mihLines(ii)=plot(mihAx,real(z),imag(z),style,'Color',colors(ii,:), ...
                'DisplayName',gain_text(curveNames{ii},curveGains{ii}),'LineWidth',1.3);
        end
        xlim(mihAx,[-1.1 1.1]); ylim(mihAx,[-1.1 1.1]);
        xlabel(mihAx,'Re D / (1+|D|)'); ylabel(mihAx,'Im D / (1+|D|)');
        title(mihAx,{'Годограф Михайлова, радиальное сжатие','D = p(jω) / max(1, |p(0)|), ω от 0 вверх'});
        legend(mihAx,'Location','southoutside','FontSize',8,'Interpreter','none');
    end
    function render_map()
        cla(mapAx,'reset'); hold(mapAx,'on'); grid(mapAx,'on');
        gx=cellfun(@(g)g.Kn,curveGains); gy=cellfun(@(g)g.Kw,curveGains);
        domainKey=sprintf('%s_%.9g_%.9g',kind,P.actuator.Omega,P.actuator.zeta);
        slices=unique(cell2mat(cellfun(@(g)[g.Ki;g.Ktheta],curveGains,'UniformOutput',false)).','rows');
        domainKey=[domainKey sprintf('_%.9g',slices(:))];
        if isKey(domainCache,domainKey), bounds=domainCache(domainKey);
        else
            [~,uniqueSlices]=unique(cell2mat(cellfun(@(g)[g.Ki;g.Ktheta],curveGains,'UniformOutput',false)).','rows');
            bounds=ACS.map_domain(P,kind,curveGains(uniqueSlices));
            if domainCache.Count>30, domainCache=containers.Map(); end
            domainCache(domainKey)=bounds;
        end
        bounds=[min(bounds(1),min(gx)-.2),max(bounds(2),max(gx)+.2), ...
            min(bounds(3),min(gy)-.1),max(bounds(4),max(gy)+.1)];
        xx=linspace(bounds(1),bounds(2),101); yy=linspace(bounds(3),bounds(4),81);
        maps=cell(size(curveGains));
        for ii=1:numel(curveGains)
            g=curveGains{ii}; key=sprintf('%s_%.9g_%.9g_%.9g_%.9g',kind, ...
                P.actuator.Omega,P.actuator.zeta,g.Ki,g.Ktheta);
            key=[key sprintf('_%.9g',bounds)];
            if isKey(mapCache,key), S=mapCache(key);
            else
                S=ACS.stability_map(P,kind,g,xx,yy);
                if mapCache.Count>40, mapCache=containers.Map(); end
                mapCache(key)=S;
            end
            maps{ii}=S;
        end
        S=maps{end};
        h=imagesc(mapAx,xx,yy,double(S.stable),'HandleVisibility','off');
        h.AlphaData=.22; colormap(mapAx,[1 .83 .83;.6 .9 .7]); clim(mapAx,[0 1]); mapAx.YDir='normal';
        mapPoints=gobjects(1,numel(curveGains));
        for ii=1:numel(curveGains)
            S=maps{ii}; style='-'; if ii==numel(curveGains), style='--'; end
            if min(S.margin(:))<0 && max(S.margin(:))>0
                contour(mapAx,xx,yy,S.margin,[0 0],style,'Color',colors(ii,:), ...
                    'LineWidth',1.2,'HandleVisibility','off');
            end
            mapPoints(ii)=plot(mapAx,gx(ii),gy(ii),'o','Color',colors(ii,:), ...
                'MarkerFaceColor',colors(ii,:),'DisplayName',gain_text(curveNames{ii},curveGains{ii}));
        end
        xline(mapAx,0,':','HandleVisibility','off'); yline(mapAx,0,':','HandleVisibility','off');
        xlim(mapAx,bounds(1:2)); ylim(mapAx,bounds(3:4)); xlabel(mapAx,'Kn'); ylabel(mapAx,'Kw');
        title(mapAx,{'Области по Гурвицу, включая отрицательные K','Зелёная заливка для текущих Ki и Kθ'});
        legend(mapAx,'Location','southoutside','FontSize',8,'Interpreter','none');
    end
    function export_results()
        folder=uigetdir(root,'Папка для графиков и моделей выбранной системы');
        if isequal(folder,0), return; end
        status.Text='Сохранение графиков и моделей'; drawnow;
        for jj=1:3, exportgraphics(ax(jj),fullfile(folder,sprintf('%s_response_%d.png',kind,jj)),'Resolution',180); end
        exportgraphics(mapAx,fullfile(folder,[kind '_stability.png']),'Resolution',180);
        exportgraphics(mihAx,fullfile(folder,[kind '_mikhailov.png']),'Resolution',180);
        config=P; result=R; current=candidate; comparison_gains=curveGains; comparison_names=curveNames;
        save(fullfile(folder,[kind '_calculation.mat']),'config','result','current','comparison_gains','comparison_names');
        modelGains={initial,P.reference.(kind),candidate}; modelNames={'initial','reference','current'};
        if ~isempty(R), modelGains{end+1}=R.refined; modelNames{end+1}='refined'; end
        for jj=1:numel(modelGains)
            Q=ACS.apply_settings(P,kind,modelGains{jj}); Q.t_end=tEnd;
            ACS.build_simulink(Q,fullfile(folder,['models_' modelNames{jj}]),{kind},'.slx',true,['_' modelNames{jj}]);
        end
        status.Text='Сохранены 5 графиков, параметры расчёта и модели Simulink выбранной системы.';
    end
end
