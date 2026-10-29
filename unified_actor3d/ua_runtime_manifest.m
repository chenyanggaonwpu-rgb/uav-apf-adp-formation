function manifest=ua_runtime_manifest()
    project=fileparts(fileparts(mfilename('fullpath')));
    folders={'','config','src','src3d','neural_adp3d','neural_adp_supervisor', ...
        'five_uav_actor3d','five_uav_comparison3d','cpu_realtime_adp3d','unified_actor3d'};
    entries=struct('path',{},'sha256',{});
    for k=1:numel(folders)
        files=dir(fullfile(project,folders{k},'*.m'));
        for j=1:numel(files)
            rel=strrep(fullfile(folders{k},files(j).name),'\','/');
            entries(end+1)=struct('path',rel,'sha256',ua_sha256(fullfile(project,rel))); %#ok<AGROW>
        end
    end
    [~,order]=sort({entries.path});entries=entries(order);
    digest=java.security.MessageDigest.getInstance('SHA-256');
    for k=1:numel(entries),digest.update(unicode2native(sprintf('%s:%s\n',entries(k).path,entries(k).sha256),'UTF-8'));end
    fingerprint=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
    manifest=struct('sha256',fingerprint,'entries',entries);
end
