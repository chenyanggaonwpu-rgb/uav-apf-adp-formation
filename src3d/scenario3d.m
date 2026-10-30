function scene=scenario3d(id,seed,p)
% SCENARIO3D Predeclared fixed maps and reproducible random maps.
    if nargin<2 || isempty(seed), seed=0; end
    if nargin<3, p=default_params3d(); end
    empty=struct('type',{},'center',{},'r',{},'zmin',{},'zmax',{},'halfSize',{});
    scene=struct('id',id,'kind','fixed','seed',seed,'obstacles',empty, ...
        'family','legacy_fixed','description','');
    switch id
        case 'S0_empty'
        case 'S1_sphere'
            scene.obstacles=sphere([40;36;10],4);
        case 'S2_cylinder'
            scene.obstacles=cylinder([40;36],5,0,16);
        case 'S3_overpass'
            for y=0:8:80
                scene.obstacles(end+1)=cylinder([40;y],4,0,13); %#ok<AGROW>
            end
        case 'S4_space_s'
            scene.obstacles=[sphere([29;26;10],4), ...
                cylinder([43;39],4,0,17),sphere([54;46;17],4)];
        case 'S5_choice'
            scene.obstacles=[cylinder([35;35],5,0,18), ...
                sphere([47;35;11],4),sphere([52;50;19],4)];
        case 'S6_terminal'
            scene.obstacles=[sphere([37;33;10],4),sphere([65;58;13],2)];
        case 'S7_underpass'
            scene.family='underpass';
            scene.description='Full-width roof: pass below, not above.';
            scene.obstacles=obstacle3d('box',[36;0;14.5],[44;80;30]);
        case 'S8_window'
            scene.family='window';
            scene.description='Full-width wall with a 10-20 m vertical opening; only base layer 15 lies strictly inside it.';
            scene.obstacles=[obstacle3d('box',[36;0;0],[44;80;10]), ...
                obstacle3d('box',[36;0;20],[44;80;30])];
        case 'S9_altitude_choice'
            scene.family='altitude_choice';
            scene.description='Choose a horizontal detour or climb below a ceiling.';
            scene.obstacles=[obstacle3d('box',[34;10;0],[46;70;13]), ...
                obstacle3d('box',[34;10;29],[46;70;30])];
        case 'S10_layer_gap'
            scene.family='layer_gap';
            scene.description='Full-width wall with an 8-18 m opening; layer coverage depends on planning clearance.';
            scene.obstacles=[obstacle3d('box',[36;0;0],[44;80;8]), ...
                obstacle3d('box',[36;0;18],[44;80;30])];
        case 'S11_blocked'
            scene.family='blocked_control';
            scene.description='Solid wall spans the entire bounded cross-section; no continuous crossing exists.';
            scene.obstacles=obstacle3d('box',[36;0;0],[44;80;30]);
        case {'random','random_normal','normal_random'}
            scene=normal_scene3d(seed,p);
        case 'random_unconditioned'
            scene=random_solids3d(seed,p);
        case 'random_structured'
            scene=random_mixed(scene,seed,p);
        case 'random_legacy'
            scene.kind='random_legacy'; scene.family='legacy_open_edges';
            scene.description='Original 2.0.0 sampling geometry; use frozen 2.0.0 sources for exact historical reproduction.';
            rng(seed,'twister');
            for i=1:p.randomObstacleCount
                placed=false;
                for attempt=1:300
                    xy=[15+50*rand;15+50*rand];
                    if rand<0.5
                        o=sphere([xy;6+18*rand],2+2*rand);
                    else
                        o=cylinder(xy,2+2*rand,0,10+10*rand);
                    end
                    if obstacle_distance3d(p.start,o)<5.6 || ...
                            obstacle_distance3d(p.target,o)<5.6
                        continue;
                    end
                    separated=true;
                    for j=1:numel(scene.obstacles)
                        other=scene.obstacles(j);
                        if norm(xy-other.center(1:2))<o.r+other.r+1
                            separated=false; break;
                        end
                    end
                    if separated
                        scene.obstacles(end+1)=o; %#ok<AGROW>
                        placed=true; break;
                    end
                end
                if ~placed
                    scene.kind='generation_failed';
                    return;
                end
            end
        otherwise
            error('UAV3D:UnknownScene','Unknown 3-D scene: %s',id);
    end
end

function o=sphere(center,r)
    o=obstacle3d('sphere',center,r);
end

function o=cylinder(center,r,zmin,zmax)
    o=obstacle3d('cylinder',center,r,zmin,zmax);
end

function scene=random_mixed(scene,seed,p)
% Four predeclared map families; no graph or flight success filtering.
    scene.kind='random'; rng(seed,'twister');
    families={'mixed','underpass','window','staggered_windows'};
    familyIndex=mod(seed-1,4)+1;
    if p.randomObstacleCount<4, familyIndex=1; end
    scene.family=families{familyIndex};
    scene.description='Mixed 3-D solids; family fixed by seed modulo four; endpoints reserved, path feasibility not screened.';
    if familyIndex==2
        scene.obstacles=obstacle3d('box',[36;0;12+4*rand],[44;80;30]);
    elseif familyIndex==3
        z=11+8*rand;
        scene.obstacles=window_wall(35,45,z);
    elseif familyIndex==4
        scene.obstacles=[window_wall(28,34,11+8*rand), ...
            window_wall(49,55,11+8*rand)];
    end
    [initial,offsets,psi]=init_formation3d(p);
    rot=[cos(psi) -sin(psi);sin(psi) cos(psi)];
    terminal=[p.target(1:2)+rot*offsets;p.target(3)*ones(1,p.nUAV)];
    endpoints=[initial(1:3,:),terminal];
    for i=numel(scene.obstacles)+1:p.randomObstacleCount
        placed=false;
        for attempt=1:300
            xy=[5+70*rand;5+70*rand];
            r=2+2*rand;
            switch mod(i-1,4)
                case 0
                    o=sphere([xy;r+(30-2*r)*rand],r);
                case 1
                    o=cylinder(xy,r,0,12+18*rand);
                case 2
                    zlo=8+10*rand;
                    o=cylinder(xy,r,zlo,min(30,zlo+4+8*rand));
                case 3
                    half=[2+3*rand;2+5*rand;2+4*rand];
                    xy=max(half(1:2),min(80-half(1:2),xy));
                    c=[xy;half(3)+(30-2*half(3))*rand];
                    o=obstacle3d('box',c-half,c+half);
            end
            safe=true;
            for e=1:size(endpoints,2)
                if obstacle_distance3d(endpoints(:,e),o)<p.desiredClear
                    safe=false; break;
                end
            end
            if ~safe || obstacle_distance3d(p.start,o)<5.6 || ...
                    obstacle_distance3d(p.target,o)<5.6
                continue;
            end
            [lo,hi]=bounds(o);
            separated=true;
            for j=1:numel(scene.obstacles)
                [otherLo,otherHi]=bounds(scene.obstacles(j));
                gap=max(max(lo-otherHi,otherLo-hi),0);
                if norm(gap)<1, separated=false; break; end
            end
            if separated
                scene.obstacles(end+1)=o; %#ok<AGROW>
                placed=true; break;
            end
        end
        if ~placed, scene.kind='generation_failed'; return; end
    end
end

function obstacles=window_wall(xlo,xhi,z)
    obstacles=[obstacle3d('box',[xlo;0;0],[xhi;80;z-5]), ...
        obstacle3d('box',[xlo;0;z+5],[xhi;80;30])];
end

function [lo,hi]=bounds(o)
    switch o.type
        case 'sphere'
            lo=o.center-o.r; hi=o.center+o.r;
        case 'cylinder'
            lo=[o.center-o.r;o.zmin]; hi=[o.center+o.r;o.zmax];
        case 'box'
            lo=o.center-o.halfSize; hi=o.center+o.halfSize;
    end
end
