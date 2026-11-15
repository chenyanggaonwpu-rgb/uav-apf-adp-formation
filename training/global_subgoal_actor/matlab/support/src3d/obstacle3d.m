function o=obstacle3d(type,varargin)
% OBSTACLE3D Create a sphere, closed vertical cylinder or axis-aligned box.
% obstacle3d('box', lowerXYZ, upperXYZ) also represents roofs and wall slabs.
    o=struct('type',type,'center',[],'r',nan,'zmin',nan,'zmax',nan, ...
        'halfSize',nan(3,1));
    switch type
        case 'sphere'
            c=varargin{1}(:); r=varargin{2};
            validateattributes(c,{'numeric'},{'size',[3 1],'finite'});
            validateattributes(r,{'numeric'},{'scalar','positive','finite'});
            o.center=c; o.r=r;
        case 'cylinder'
            c=varargin{1}(:); r=varargin{2}; zlo=varargin{3}; zhi=varargin{4};
            validateattributes(c,{'numeric'},{'size',[2 1],'finite'});
            validateattributes(r,{'numeric'},{'scalar','positive','finite'});
            validateattributes(zlo,{'numeric'},{'scalar','finite'});
            validateattributes(zhi,{'numeric'},{'scalar','finite','>',zlo});
            o.center=c; o.r=r; o.zmin=zlo; o.zmax=zhi;
        case 'box'
            lo=varargin{1}(:); hi=varargin{2}(:);
            validateattributes(lo,{'numeric'},{'size',[3 1],'finite'});
            validateattributes(hi,{'numeric'},{'size',[3 1],'finite'});
            if any(hi<=lo), error('UAV3D:BoxSize','Box upper corner must exceed lower corner.'); end
            o.center=(lo+hi)/2; o.halfSize=(hi-lo)/2;
            o.zmin=lo(3); o.zmax=hi(3);
        otherwise
            error('UAV3D:ObstacleType','Unsupported obstacle type: %s',type);
    end
end
