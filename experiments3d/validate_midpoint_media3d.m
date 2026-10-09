function validate_midpoint_media3d(studyDir)
% VALIDATE_MIDPOINT_MEDIA3D Decode the delivered movie for visual review.
    example=jsondecode(fileread(fullfile(studyDir,'example_selection.json')));
    movie=fullfile(studyDir,'examples',sprintf('seed_%06d',example.seed),'flight_3d.mp4');
    video=VideoReader(movie);
    assert(video.Width>=1000 && video.Height>=700 && video.Duration>0 && video.FrameRate>0);
    folder=fullfile(studyDir,'validation','media'); if ~isfolder(folder),mkdir(folder);end
    fractions=[0.05 0.5 0.95]; records=struct([]);
    for i=1:numel(fractions)
        video.CurrentTime=min(video.Duration*fractions(i),max(video.Duration-1/video.FrameRate,0));
        timestamp=video.CurrentTime; pixels=readFrame(video);
        assert(~isempty(pixels) && size(pixels,3)==3);
        name=sprintf('flight_frame_%02d.png',i); imwrite(pixels,fullfile(folder,name));
        records=[records;struct('frame',name,'timeSeconds',timestamp)]; %#ok<AGROW>
    end
    receipt=struct('seed',example.seed,'source',movie,'width',video.Width,'height',video.Height, ...
        'duration',video.Duration,'frameRate',video.FrameRate,'decodedFrames',records);
    fid=fopen(fullfile(folder,'decode_receipt.json'),'w','n','UTF-8');
    fprintf(fid,'%s',jsonencode(receipt,PrettyPrint=true));fclose(fid);
    fprintf('MIDPOINT_MEDIA_DECODED seed=%d width=%d height=%d duration=%.3f fps=%.3f frames=3\n', ...
        example.seed,video.Width,video.Height,video.Duration,video.FrameRate);
end
