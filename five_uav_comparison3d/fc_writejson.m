function fc_writejson(path,value)
    fid=fopen(path,'w','n','UTF-8');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s',jsonencode(value,PrettyPrint=true));
end
