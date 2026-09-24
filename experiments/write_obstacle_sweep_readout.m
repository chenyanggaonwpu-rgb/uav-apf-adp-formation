function write_obstacle_sweep_readout(outputDir)
% WRITE_OBSTACLE_SWEEP_READOUT Factual, automatically reproducible readout.
    x=load(fullfile(outputDir,'experiment.mat'),'cfg'); cfg=x.cfg;
    rows=readtable(fullfile(outputDir,'trials.csv'),'TextType','string');
    s=obstacle_sweep_summary(rows,cfg.combinations,cfg.seeds);
    fid=fopen(fullfile(outputDir,'RESULTS.md'),'w','n','UTF-8');
    assert(fid>=0,'UAV:SweepWrite','Cannot write results readout.');
    closer=onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid,'# 障碍环境三参数扫描结果\n\n');
    if height(rows)<height(cfg.jobs)
        fprintf(fid,'**阶段性结果：试验尚未完成，不用于最终比较。**\n\n');
    end
    fprintf(fid,'已完成 %d/%d 次计划试验；共 %d 个参数组合，每组计划 %d 个 seed。\n\n', ...
        height(rows),height(cfg.jobs),height(s),numel(cfg.seeds));
    fprintf(fid,'固定最大速度 %.1f m/s、规划膨胀净空 %.1f m、场地 %.0f×%.0f m。\n\n', ...
        cfg.baseParams.vmax,cfg.baseParams.pathClearance, ...
        diff(cfg.baseParams.workspace(:,1)),diff(cfg.baseParams.workspace(:,2)));
    fprintf(fid,'## 试验计数\n\n|结果|数量|\n|---|---:|\n');
    categories=["success","collision","timeout","invalid_state","generation_failed","execution_error"];
    labels={'成功','碰撞','超时','无效状态','场景生成失败','程序执行异常'};
    for i=1:numel(categories), fprintf(fid,'|%s|%d|\n',labels{i},sum(rows.outcome==categories(i))); end
    fprintf(fid,'\n各组合的飞行成功率 = 成功数 / 已生成且完成判定的场景数。');
    fprintf(fid,'整体成功比例还将生成失败纳入分母；执行异常出现时该组成功率不予报告。\n\n');
    fprintf(fid,'## 固定另外两项参数时的关系\n\n');
    keys={'nObstacles','corridorBand','marginObs'};
    names={'障碍物数量','两侧散布半宽 / m','最小边缘间距 / m'};
    levels=cell(1,3); fixed=zeros(1,3);
    for i=1:3, levels{i}=unique(s.(keys{i})); fixed(i)=levels{i}(ceil(end/2)); end
    for a=1:3
        fprintf(fid,'### %s\n\n',names{a});
        mask=true(height(s),1);
        for b=1:3
            if b~=a
                mask=mask & s.(keys{b})==fixed(b);
                fprintf(fid,'固定 %s = %g。',names{b},fixed(b));
            end
        end
        t=sortrows(s(mask,:),keys{a});
        fprintf(fid,'\n\n|参数值|成功/已生成|飞行成功率|Wilson 95%% 区间|生成失败/已执行|成功平均时间 / s|\n');
        fprintf(fid,'|---:|---:|---:|---:|---:|---:|\n');
        for k=1:height(t)
            fprintf(fid,'|%g|%d/%d|%.1f%%|[%.2f%%, %.2f%%]|%d/%d|%.2f|\n', ...
                t.(keys{a})(k),t.successes(k),t.generated(k),t.successPct(k), ...
                t.ci95LowPct(k),t.ci95HighPct(k),t.generationFailures(k), ...
                t.completed(k),t.meanSuccessTime_s(k));
        end
        fprintf(fid,'\n');
    end
    fprintf(fid,'## 图表与数据\n\n');
    fprintf(fid,'- `figures/01_factor_effects`：固定其它两项的成功率与 95%% 区间。\n');
    fprintf(fid,'- `figures/02_flight_success_heatmaps`：125 组的条件飞行成功率。\n');
    fprintf(fid,'- `figures/03_overall_success_heatmaps`：包含生成失败的整体成功比例。\n');
    fprintf(fid,'- `figures/04_generation_failure_heatmaps`：场景生成失败率。\n');
    fprintf(fid,'- `figures/05_completion_time_heatmaps`：仅成功样本的平均完成时间。\n');
    fprintf(fid,'- `figures/06_three_factor_scatter`：三参数位置、颜色表示成功率。\n');
    fprintf(fid,'- `figures/07_interaction_curves`：不同散布宽度下的数量关系曲线。\n');
    fprintf(fid,'- 图表均输出 PNG 和矢量 PDF；`summary.csv` 保存每组分母、置信区间和指标。\n');
    fprintf(fid,'- `trials.csv` 保存每个组合×seed 的结果；`source_snapshot/` 与参数快照可复现。\n\n');
    fprintf(fid,'## 解释边界\n\n');
    fprintf(fid,'1. 每组 200 个 seed 用于趋势筛选，细微差异不自动代表统计显著。95%% 区间为逐组区间。\n');
    fprintf(fid,'2. 场景仍经过原程序的可行性筛选；生成失败指耗尽重试，不能解释为原始地图不可行概率。\n');
    fprintf(fid,'3. 相同 seed 在不同参数下不保证相同地图；不作严格配对推断。\n');
    fprintf(fid,'4. 散布半宽 B 为航线左右各 B m，越界采样会被截断；实际障碍间距也不等于所设下限。\n');
    fprintf(fid,'5. 不跨不同参数组合混用样本来缩窄单组区间。最高观测成功率的环境需独立新 seed 复核。\n');
    fprintf(fid,'6. 本轮改变环境难度，不能据此判断控制算法升级的因果收益，也不代表真实飞行成功率。\n');
end
