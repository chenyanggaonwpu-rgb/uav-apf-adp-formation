function manifest = adp3d_source_manifest(cfg)
% 对实际运行的原工程和本模块源码取 SHA-256，恢复训练时核对版本。
    manifest=struct('module',scan(cfg.moduleRoot,{''}), ...
        'project',scan(cfg.projectRoot,{'','config','src','src3d'}));
end

function entries=scan(root,folders)
    entries=struct('path',{},'sha256',{});
    for k=1:numel(folders)
        files=dir(fullfile(root,folders{k},'*.m'));
        for j=1:numel(files)
            relative=strrep(fullfile(folders{k},files(j).name),'\','/');
            fid=fopen(fullfile(root,folders{k},files(j).name),'rb');
            assert(fid>=0); cleanup=onCleanup(@() fclose(fid));
            bytes=fread(fid,inf,'*uint8');
            digest=java.security.MessageDigest.getInstance('SHA-256');
            digest.update(bytes);
            hash=typecast(digest.digest(),'uint8');
            entries(end+1)=struct('path',relative, ...
                'sha256',lower(reshape(dec2hex(hash,2).',1,[]))); %#ok<AGROW>
            clear cleanup
        end
    end
    if ~isempty(entries), [~,order]=sort({entries.path}); entries=entries(order); end
end
