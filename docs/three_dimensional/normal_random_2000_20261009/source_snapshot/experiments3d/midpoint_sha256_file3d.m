function digest=midpoint_sha256_file3d(file)
    fid=fopen(file,'rb'); assert(fid>=0); guard=onCleanup(@() fclose(fid));
    md=java.security.MessageDigest.getInstance('SHA-256');
    while ~feof(fid), md.update(fread(fid,1048576,'*uint8')); end
    digest=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
end
