function ensure= factor_study_page(parent,P)
% Отдельная страница. Расчёт ленивый и не связан с ползунками основного окна.
S=[]; studyKind='static_ny'; savedStudies=containers.Map();
layout=uigridlayout(parent,[4 1]); layout.RowHeight={36,65,'1x',40};
controls=uigridlayout(layout,[1 4]); controls.ColumnWidth={280,150,120,'1x'}; controls.Padding=[0 0 0 0];
mode=uidropdown(controls,'Items',{'Статическая перегрузка','Астатическая перегрузка'}, ...
    'ItemsData',{'static_ny','astatic_ny'},'Tag','factor_mode','ValueChangedFcn',@change_mode);
uilabel(controls,'Text','Фиксированный Ki');
kiBox=uieditfield(controls,'numeric','Limits',[.001 100],'Value',P.reference.astatic_ny.Ki, ...
    'Enable','off','Tag','factor_Ki','ValueChangedFcn',@(~,~)calculate(false));
uilabel(controls,'Text','Ki по умолчанию из таблицы 2');
description=uilabel(layout,'Text',['Статический контур перегрузки, вариант 6. Меняется только один параметр. ' ...
    'Для c₁, c₂, c₉ привод Ω = 20 рад/с. Для частоты Ω = 10, 15, 20 рад/с. ' ...
    'Во всех случаях ξ = 0,8. Настройки основного окна не изменяются.'],'WordWrap','on');
pages=uitabgroup(layout); basePage=uitab(pages,'Title','Влияние c и привода');
kiPage=uitab(pages,'Title','Влияние Ki');
plots=uigridlayout(basePage,[2 2]); axesList=gobjects(1,5);
for i=1:4
    axesList(i)=uiaxes(plots,'Tag',sprintf('factor_axis_%d',i));
    axesList(i).Layout.Row=ceil(i/2); axesList(i).Layout.Column=mod(i-1,2)+1;
end
kiLayout=uigridlayout(kiPage,[2 1]); kiLayout.RowHeight={45,'1x'};
uilabel(kiLayout,'Text','Для астатического контура сравниваются Ki = 0,5Ki₀, Ki₀, 1,5Ki₀. Точки показывают только координаты Kn и Kw, Ki задаётся каждой кривой.', ...
    'WordWrap','on');
axesList(5)=uiaxes(kiLayout,'Tag','factor_axis_5');
buttons=uigridlayout(layout,[1 3]); buttons.ColumnWidth={160,210,'1x'}; buttons.Padding=[0 0 0 0];
uibutton(buttons,'Text','Пересчитать','ButtonPushedFcn',@(~,~)calculate(true));
saveButton=uibutton(buttons,'Text','Сохранить четыре графика','Enable','off','ButtonPushedFcn',@save_plots);
status=uilabel(buttons,'Text','Графики рассчитываются при открытии вкладки.','WordWrap','on');
ensure=@()calculate(false);

    function change_mode(~,~)
        studyKind=mode.Value;
        kiBox.Enable=matlab.lang.OnOffSwitchState(strcmp(studyKind,'astatic_ny'));
        if strcmp(studyKind,'static_ny'), pages.SelectedTab=basePage; end
        calculate(false);
    end
    function calculate(force)
        key=sprintf('%s_%.12g',studyKind,kiBox.Value);
        status.Text='Расчёт областей по критерию Гурвица'; drawnow;
        if ~force && isKey(savedStudies,key), S=savedStudies(key);
        else
            if strcmp(studyKind,'static_ny'), S=lr1.static_factor_study(P);
            else, S=lr1.astatic_factor_study(P,kiBox.Value); end
            if savedStudies.Count>12, savedStudies=containers.Map(); end
            savedStudies(key)=S;
        end
        isAst=strcmp(studyKind,'astatic_ny');
        if isAst
            contourName='Астатический контур ny';
            description.Text=sprintf(['Вариант 6. Для c и Ω фиксирован Ki = %.4g. ' ...
                'Отдельная страница показывает влияние Ki. Для c и Ki привод Ω = 20 рад/с, ξ = 0,8. ' ...
                'Точки СК и методички показывают только Kn и Kw при Ki выбранной серии. Основная модель не меняется.'],kiBox.Value);
        else
            contourName='Статический контур ny';
            description.Text='Вариант 6. Меняется только один параметр. Для c привод Ω = 20 рад/с, ξ = 0,8. Частоты 10, 15, 20 рад/с. Основная модель не меняется.';
            cla(axesList(5),'reset'); title(axesList(5),'Ki отсутствует в статическом контуре');
        end
        colors=[0 .447 .741; .85 .325 .098; 0 .5 .2];
        for ii=1:numel(S.panels)
            ax=axesList(ii); cla(ax,'reset'); ax.Tag=sprintf('factor_axis_%d',ii);
            hold(ax,'on'); grid(ax,'on');
            panel=S.panels(ii);
            for jj=1:3
                m=panel.maps{jj};
                [~,h]=contour(ax,m.x,m.y,m.margin,[0 0],'LineColor',colors(jj,:),'LineWidth',1.6);
                name=sprintf('%s = %.4g',panel.field,panel.values(jj));
                if strcmp(panel.field,'Omega'), name=sprintf('Ω = %g рад/с',panel.values(jj)); end
                if m.touches_edge, name=[name ' (продолжение за рамкой)']; end
                h.DisplayName=name;
            end
            plot(ax,S.gains.Kn,S.gains.Kw,'ko','MarkerFaceColor','k','MarkerSize',6, ...
                'DisplayName',sprintf('СК, Kn=%.4g Kw=%.4g',S.gains.Kn,S.gains.Kw));
            plot(ax,S.reference.Kn,S.reference.Kw,'kd','MarkerFaceColor','w','MarkerSize',7, ...
                'DisplayName',sprintf('Методичка, Kn=%.4g Kw=%.4g',S.reference.Kn,S.reference.Kw));
            xline(ax,0,':','HandleVisibility','off'); yline(ax,0,':','HandleVisibility','off');
            xlabel(ax,'K_{ny}'); ylabel(ax,'K_{\omega_z}');
            titles={'Влияние c₂','Влияние c₁','Влияние c₉','Влияние частоты привода','Влияние Ki'};
            title(ax,{titles{ii},[contourName ', ξ = 0,8']});
            sub=''; if ii~=4, sub='Ω = 20 рад/с'; end
            if isAst && ii~=5, sub=sprintf('%s   Ki = %.4g (для всех трёх кривых)',sub,kiBox.Value); end
            subtitle(ax,sub);
            xlim(ax,panel.bounds(1:2)); ylim(ax,panel.bounds(3:4));
            legend(ax,'Location','southoutside','Interpreter','none','FontSize',10);
        end
        setappdata(parent,'factor_study',S); saveButton.Enable='on';
        saveButton.Text=sprintf('Сохранить графики (%d)',numel(S.panels));
        status.Text='Граница рамки не замыкает область. Отрицательные K показаны для полного обзора.';
    end
    function save_plots(~,~)
        folder=uigetdir(pwd,'Папка для графиков влияния параметров');
        if isequal(folder,0), return; end
        for ii=1:numel(S.panels)
            exportgraphics(axesList(ii),fullfile(folder,[studyKind '_factor_' S.panels(ii).field '.png']),'Resolution',200);
        end
        study=S; save(fullfile(folder,[studyKind '_factor_study.mat']),'study');
        status.Text=sprintf('Сохранены графики (%d) и исходные данные экспериментов.',numel(S.panels));
    end
end
