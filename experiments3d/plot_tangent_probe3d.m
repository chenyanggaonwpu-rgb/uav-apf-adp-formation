function plot_tangent_probe3d(outputDir)
% PLOT_TANGENT_PROBE3D Geometric APF probe, not a flown trajectory.
    root=fileparts(fileparts(mfilename('fullpath')));
    oldPath=path;
    cleanup=onCleanup(@() path(oldPath)); %#ok<NASGU>
    addpath(fullfile(root,'config'),fullfile(root,'src'),fullfile(root,'src3d'));
    if nargin<1 || isempty(outputDir)
        outputDir=fullfile(root,'docs','three_dimensional','assets','apf_probe');
    end
    if ~isfolder(outputDir), mkdir(outputDir); end
    p=default_params3d();
    % The fallback target is radially aligned only at the probe centre.
    p.target=[50;36;10];
    sphereScene=scenario3d('S1_sphere',p.seed,p);
    cylinderScene=scenario3d('S2_cylinder',p.seed,p);
    t=linspace(-2.5,2.5,101);
    spherePoints=[44.5*ones(size(t));36+t;10*ones(size(t))];
    cylinderPoints=[45.7*ones(size(t));36+t;17+1.6*t];
    [sphereForce,sphereRisk,sphereDegenerate]=probe(spherePoints, ...
        [40;36;10],sphereScene.obstacles,p);
    [cylinderForce,cylinderRisk,cylinderDegenerate]=probe(cylinderPoints, ...
        [40;36;10],cylinderScene.obstacles,p);
    assert(all(isfinite(sphereForce),'all') && ...
        all(isfinite(cylinderForce),'all'));
    names={'probe','x','y','z','fx','fy','fz','risk','degenerate'};
    rows=[ones(numel(t),1),spherePoints',sphereForce',sphereRisk', ...
        double(sphereDegenerate'); ...
        2*ones(numel(t),1),cylinderPoints',cylinderForce',cylinderRisk', ...
        double(cylinderDegenerate')];
    writetable(array2table(rows,'VariableNames',names), ...
        fullfile(outputDir,'tangent_probe.csv'));
    fig=figure('Visible','off','Color','w','Position',[100 100 1100 740]);
    tiledlayout(fig,2,2,'Padding','compact');
    nexttile;plot(t,sphereForce');grid on;
    legend({'F_x','F_y','F_z'},'Location','best');
    xlabel('Sphere probe y offset (m)');ylabel('APF force (m/s^2)');
    title('Sphere: radial alignment and tangent fallback');
    nexttile;plot(t,sphereRisk,'LineWidth',1.2);hold on;
    stairs(t,double(sphereDegenerate),'LineWidth',1.2);grid on;
    legend({'risk','degenerate'},'Location','best');
    xlabel('Sphere probe y offset (m)');
    nexttile;plot(cylinderPoints(3,:),cylinderForce');grid on;
    legend({'F_x','F_y','F_z'},'Location','best');
    xlabel('Cylinder probe height (m)');ylabel('APF force (m/s^2)');
    title('Cylinder: side, rim and top transition');
    nexttile;plot(cylinderPoints(3,:),cylinderRisk,'LineWidth',1.2);hold on;
    stairs(cylinderPoints(3,:),double(cylinderDegenerate),'LineWidth',1.2);
    grid on;legend({'risk','degenerate'},'Location','best');
    xlabel('Cylinder probe height (m)');
    exportgraphics(fig,fullfile(outputDir,'tangent_probe.png'),'Resolution',160);
    savefig(fig,fullfile(outputDir,'tangent_probe.fig'));
    close(fig);
    fprintf('APF tangent probe: sphere degeneracy=%d, cylinder degeneracy=%d.\n', ...
        nnz(sphereDegenerate),nnz(cylinderDegenerate));
end

function [force,risk,degenerate]=probe(points,goal,obstacles,p)
    count=size(points,2);
    force=zeros(3,count);risk=zeros(1,count);degenerate=false(1,count);
    for i=1:count
        [force(:,i),risk(i),degenerate(i)]= ...
            apf_repulsion3d(points(:,i),goal,obstacles,p);
    end
end
