function value=normal_seed_study_io3d(action,varargin)
% NORMAL_SEED_STUDY_IO3D Atomic checkpoints and canonical geometry hashes.
    value=[];
    switch action
        case 'json'
            file=varargin{1}; temporary=[file,'.tmp'];
            fid=fopen(temporary,'w','n','UTF-8');assert(fid>=0);
            fprintf(fid,'%s',jsonencode(varargin{2},PrettyPrint=true));fclose(fid);
            movefile(temporary,file,'f');
        case 'table'
            file=varargin{1};temporary=[file,'.tmp.csv'];
            writetable(varargin{2},temporary);movefile(temporary,file,'f');
        case 'map_hash'
            q=varargin{1};scene=varargin{2};
            bytes=unicode2native(jsonencode(struct('workspace',q.workspace,'obstacles',scene.obstacles)),'UTF-8');
            md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);
            value=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
        otherwise
            error('UAV3D:StudyIO','Unknown action %s.',action);
    end
end
