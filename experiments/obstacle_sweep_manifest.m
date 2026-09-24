function files = obstacle_sweep_manifest(root)
% OBSTACLE_SWEEP_MANIFEST SHA-256 of the source used by an experiment.
    paths = dir(fullfile(root, '**', '*.m'));
    relativePath = strings(numel(paths),1);
    sha256 = strings(numel(paths),1);
    for i=1:numel(paths)
        full = fullfile(paths(i).folder,paths(i).name);
        relativePath(i) = string(full(numel(root)+2:end));
        fid=fopen(full,'rb');
        if fid<0, error('UAV:SweepRead','Cannot read %s',full); end
        closer=onCleanup(@() fclose(fid));
        bytes=fread(fid,Inf,'*uint8');
        clear closer;
        digest=java.security.MessageDigest.getInstance('SHA-256');
        digest.update(bytes);
        raw=typecast(digest.digest(),'uint8');
        sha256(i)=string(lower(reshape(dec2hex(raw,2).',1,[])));
    end
    files=sortrows(table(relativePath,sha256),'relativePath');
end
