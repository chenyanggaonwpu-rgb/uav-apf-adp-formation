function validate_margin_cache3d(studyDir)
% VALIDATE_MARGIN_CACHE3D Exact state/reference equivalence, not a timing test.
    snap=fullfile(studyDir,'source_snapshot');
    addpath(fullfile(snap,'config'),fullfile(snap,'src3d'),fullfile(snap,'experiments3d'));
    protocol=jsondecode(fileread(fullfile(studyDir,'protocol.json')));
    old=readtable(fullfile(protocol.baseStudy,'all_results.csv'),'TextType','string');
    pathSeed=min(old.seed(old.success==1 & old.planStatus=="path"));
    cases=[2001 .8 .8;pathSeed 4.2 4;2003 5.6 4.4];records=struct([]);
    for i=1:size(cases,1)
        d=load(fullfile(studyDir,'maps',sprintf('seed_%06d.mat',cases(i,1))));
        q=d.q;q.pathClearance=cases(i,2);q.desiredClear=cases(i,3);
        q.directionalClearance=struct('enable',false,'planningZ',q.pathClearance,'referenceZ',q.desiredClear);
        [~,~,cache]=plan_layers3d(q.start,q.target,d.scene.obstacles,q,q.pathClearance,q.pathClearance,[]);
        plain=run_simulation3d(q,d.scene);warm=run_simulation3d(q,d.scene,cache);
        for field={'success','reason','time','state','route'}
            assert(isequaln(plain.(field{1}),warm.(field{1})),field{1});
        end
        hp=plain.history;hw=warm.history;
        if isfield(hp,'cycleMs'),hp=rmfield(hp,'cycleMs');hw=rmfield(hw,'cycleMs');end
        assert(isequaln(hp,hw),'UAV3D:CacheEquivalence','Cache changed trajectory or references.');
        if isfield(plain,'metrics'),assert(isequaln(plain.metrics,warm.metrics));end
        records=[records;struct('seed',q.seed,'path',q.pathClearance,'reference',q.desiredClear, ...
            'reason',string(plain.reason),'exactNonTimingHistory',true)]; %#ok<AGROW>
        fprintf('MARGIN_CACHE_VALIDATED seed=%d path=%.2f ref=%.2f reason=%s\n',q.seed,q.pathClearance,q.desiredClear,plain.reason);
    end
    value=struct('passed',true,'cases',records,'timingExcluded',true);
    fid=fopen(fullfile(studyDir,'validation','cache_equivalence.json'),'w','n','UTF-8');
    fprintf(fid,'%s',jsonencode(value,PrettyPrint=true));fclose(fid);
end
