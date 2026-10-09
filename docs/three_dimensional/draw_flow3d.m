function draw_flow3d(outputDir)
% DRAW_FLOW3D Editable MATLAB source for the actual 3-D information flow.
    if nargin<1, outputDir=fileparts(mfilename('fullpath')); end
    fig=figure('Visible','off','Color','w','Position',[100 100 1550 650]);
    blue=[0.12 0.34 0.64];green=[0.13 0.48 0.32];
    purple=[0.44 0.25 0.62];orange=[0.78 0.37 0.13];
    box(fig,[0.02 0.69 0.13 0.16],{'Known map','sphere / cylinder'},blue);
    box(fig,[0.19 0.69 0.13 0.16],{'Five-layer','planner'},blue);
    box(fig,[0.36 0.69 0.13 0.16],{'APF + V reference','base input'},blue);
    box(fig,[0.53 0.69 0.10 0.16],{'Add ADP','correction'},blue);
    box(fig,[0.67 0.69 0.10 0.16],{'3-D norm','limit'},blue);
    box(fig,[0.81 0.69 0.16 0.16],{'Predictive safety','max 5 passes'},blue);
    arrow(fig,.15,.19,.77,.77,blue);
    arrow(fig,.32,.36,.77,.77,blue);
    arrow(fig,.49,.53,.77,.77,blue);
    arrow(fig,.63,.67,.77,.77,blue);
    arrow(fig,.77,.81,.77,.77,blue);
    box(fig,[0.04 0.36 0.13 0.16],{'Measured state','position noisy'},green);
    box(fig,[0.21 0.36 0.13 0.16],{'Shared Actor','6 features / 3 axes'},purple);
    arrow(fig,.17,.21,.44,.44,purple);
    arrow(fig,.34,.58,.44,.69,purple);
    arrow(fig,.10,.42,.52,.69,green);
    box(fig,[0.52 0.36 0.13 0.16],{'External accel','after safety'},orange);
    box(fig,[0.70 0.36 0.13 0.16],{'3-D point model','Euler + speed limit'},blue);
    arrow(fig,.88,.58,.69,.52,blue);
    arrow(fig,.65,.70,.44,.44,orange);
    box(fig,[0.85 0.36 0.13 0.16],{'True state','collision metrics'},green);
    arrow(fig,.83,.85,.44,.44,blue);
    box(fig,[0.22 0.08 0.20 0.15],{'Actor-Critic update','old / new feedback + cost'},purple);
    arrow(fig,.91,.11,.36,.24,green);
    arrow(fig,.11,.11,.24,.36,green);
    arrow(fig,.15,.27,.36,.23,purple);
    arrow(fig,.32,.27,.23,.36,purple);
    annotation(fig,'textbox',[0.03 0.91 0.94 0.05], ...
        'String','3-D UAV formation: control, feedback, learning, and disturbance', ...
        'EdgeColor','none','HorizontalAlignment','center','FontSize',19, ...
        'FontWeight','bold');
    annotation(fig,'textbox',[0.03 0.01 0.94 0.04], ...
        'String','Blue: control   Green: feedback   Purple: online learning   Orange: external disturbance', ...
        'EdgeColor','none','HorizontalAlignment','center','FontSize',12);
    savefig(fig,fullfile(outputDir,'flow.fig'));
    exportgraphics(fig,fullfile(outputDir,'flow.png'),'Resolution',180);
    close(fig);
end

function box(fig,position,label,color)
    annotation(fig,'textbox',position,'String',label,'BackgroundColor', ...
        0.88+0.12*color,'EdgeColor',color,'LineWidth',1.5, ...
        'HorizontalAlignment','center','VerticalAlignment','middle', ...
        'FontSize',12,'FontWeight','bold','FitBoxToText','off');
end

function arrow(fig,x1,x2,y1,y2,color)
    annotation(fig,'arrow',[x1 x2],[y1 y2],'Color',color,'LineWidth',1.7);
end
