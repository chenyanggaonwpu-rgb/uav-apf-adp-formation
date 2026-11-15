function [p,studyDir]=clearance_candidate_params3d(studyDir)
% CLEARANCE_CANDIDATE_PARAMS3D Load the frozen selected experimental preset.
% Use replay_clearance_seed3d to reproduce maps from the recorded study.
% main3d(p) generates a new map using p.desiredClear and is a new experiment.
    if nargin<1 || isempty(studyDir)
        root=fileparts(fileparts(mfilename('fullpath')));
        studyDir=fullfile(root,'results','clearance_boundary3d_v2.2.0_20261006_203154_442');
    end
    selectionFile=fullfile(studyDir,'selection.json');
    receiptFile=fullfile(studyDir,'selection_freeze.json');
    assert(isfile(selectionFile) && isfile(receiptFile),'UAV3D:CandidateNotFrozen', ...
        'A completed candidate selection and freeze receipt are required.');
    receipt=jsondecode(fileread(receiptFile));
    fid=fopen(selectionFile,'rb'); bytes=fread(fid,inf,'*uint8'); fclose(fid);
    md=java.security.MessageDigest.getInstance('SHA-256'); md.update(bytes);
    digest=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
    assert(strcmp(digest,receipt.sha256),'UAV3D:CandidateChanged','Frozen candidate was changed.');
    selected=jsondecode(fileread(selectionFile)); c=selected.config;
    stored=load(fullfile(studyDir,'protocol.mat'),'protocol'); p=stored.protocol.p;
    p.pathClearance=c.pathXY; p.desiredClear=c.refXY;
    p.directionalClearance=struct('enable',logical(c.enabled), ...
        'planningZ',c.planZ,'referenceZ',c.refZ);
    p.noiseStd=0; p.disturbanceMax=0; p.animate=false;
    p.adp.wA=zeros(3,6); p.adp.wC=zeros(6,1);
    p.evidenceConfigId=c.configId; p.evidenceCommit=stored.protocol.sourceCommit;
    p.evidenceStudy=studyDir;
end
