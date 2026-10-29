function value=ua_sha256(file)
    fid=fopen(file,'rb');assert(fid>=0,'UA:MissingFile','Missing %s',file);
    cleanup=onCleanup(@()fclose(fid));
    digest=java.security.MessageDigest.getInstance('SHA-256');
    digest.update(fread(fid,inf,'*uint8'));
    value=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
