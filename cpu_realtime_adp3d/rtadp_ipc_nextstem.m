function stem=rtadp_ipc_nextstem(ipc)
    persistent sequence previousDirectory;
    if isempty(sequence) || ~strcmp(previousDirectory,ipc)
        sequence=0; previousDirectory=ipc;
    end
    sequence=sequence+1;
    stem=fullfile(ipc,sprintf('job_%06d',sequence));
end
