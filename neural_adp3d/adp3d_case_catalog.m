function catalog = adp3d_case_catalog(seeds,cfg,p)
% 所有声明种子均保留；不满足构造条件会报错，不静默过滤。
    cells=arrayfun(@(seed) adp3d_make_case(seed,cfg,p),seeds,'UniformOutput',false);
    catalog=[cells{:}];
    assert(numel(unique([catalog.seed]))==numel(catalog));
end
