function validate_clearance_replay3d(studyDir)
% VALIDATE_CLEARANCE_REPLAY3D Exercise the user replay entry and MP4 output.
% Run after the isolated numerical timing stage, without other MATLAB workers.
    examples=readtable(fullfile(studyDir,'examples','example_manifest.csv'),'Delimiter',',','VariableNamingRule','preserve','TextType','string');
    sample=examples(examples.kind=="staggered_windows",:);
    assert(height(sample)==1,'Expected the declared first rescued two-window example.');
    original=load(char(sample.sourceMat),'r');
    [r,folder]=replay_clearance_seed3d(sample.seed,true,studyDir);
    assert(r.success==original.r.success && strcmp(r.reason,original.r.reason));
    checks=struct([]); fields=fieldnames(original.r.history);
    for k=1:numel(fields)
        name=fields{k};
        if strcmp(name,'cycleMs'), continue; end % Wall-clock time varies.
        a=original.r.history.(name); b=r.history.(name);
        assert(isequal(size(a),size(b)),'Replay changed the size of %s.',name);
        assert(isequal(isnan(a),isnan(b)) && isequal(isinf(a),isinf(b)));
        ids=isfinite(a) & isfinite(b);
        difference=0;
        if any(ids,'all'), difference=max(abs(double(a(ids))-double(b(ids)))); end
        assert(difference<=1e-8,'Replay differs in %s by %g.',name,difference);
        checks=[checks;struct('field',string(name),'maxAbsoluteDifference',difference)]; %#ok<AGROW>
    end
    assert(max(abs(r.state-original.r.state),[],'all')<=1e-8);
    output=fullfile(studyDir,'examples','staggered_windows');
    copyfile(fullfile(folder,'flight_3d.mp4'),fullfile(output,'flight_3d.mp4'));
    copyfile(fullfile(folder,'replay_protocol.json'),fullfile(output,'replay_protocol.json'));
    reader=VideoReader(fullfile(output,'flight_3d.mp4'));
    fractions=[0.15 0.5 0.85];
    for k=1:numel(fractions)
        reader.CurrentTime=reader.Duration*fractions(k);
        frame=readFrame(reader);
        imwrite(frame,fullfile(output,sprintf('video_check_%02d.png',k)));
    end
    receipt=struct('passed',true,'seed',sample.seed,'sourceMat',sample.sourceMat, ...
        'replayFolder',folder,'fieldsCompared',{checks},'excludedTimingField','cycleMs', ...
        'videoDurationSeconds',reader.Duration,'videoFrameRate',reader.FrameRate, ...
        'videoWidth',reader.Width,'videoHeight',reader.Height);
    fid=fopen(fullfile(studyDir,'replay_validation.json'),'w','n','UTF-8');
    fprintf(fid,'%s',jsonencode(receipt,PrettyPrint=true)); fclose(fid);
    fprintf('FROZEN_REPLAY_AND_VIDEO_VALIDATED seed=%d fields=%d\n',sample.seed,numel(checks));
end
