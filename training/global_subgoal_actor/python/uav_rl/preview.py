from pathlib import Path
import json,os
import numpy as np


def preview(cases,destination,title='程序生成地图：未向控制器提供见证路线'):
    destination=Path(destination);destination.parent.mkdir(parents=True,exist_ok=True)
    cache=destination.parent/'plot_cache';cache.mkdir(exist_ok=True);os.environ['MPLCONFIGDIR']=str(cache)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle,Rectangle
    plt.rcParams['font.sans-serif']=['Microsoft YaHei','DejaVu Sans'];plt.rcParams['axes.unicode_minus']=False
    cols=4;rows=int(np.ceil(len(cases)/cols));fig,axes=plt.subplots(rows,cols,figsize=(14,rows*3.2),squeeze=False,layout='constrained')
    for ax,case in zip(axes.flat,cases):
        for o in case['obstacles']:
            color='#467b70' if o['type']=='cylinder' else '#737d8c'
            if o['type']=='box':
                c=np.asarray(o['center']);h=np.asarray(o['halfSize']);patch=Rectangle(c[:2]-h[:2],2*h[0],2*h[1],facecolor=color,alpha=.85)
            else:patch=Circle(o['center'][:2],o['r'],facecolor=color,alpha=.85)
            ax.add_patch(patch)
        ax.scatter(*case['start'][:2],c='#d58331',marker='^',s=40,zorder=5);ax.scatter(*case['goal'][:2],c='#287ca0',marker='*',s=65,zorder=5)
        upper=case['workspace'][1];ax.set(xlim=(0,upper[0]),ylim=(0,upper[1]),aspect='equal');ax.grid(alpha=.12)
        label=case['group']+f" · {len(case['obstacles'])} obstacles"
        if 'bamboo_quality' in case:label+=f"\n最长直线探测半净宽≤{case['bamboo_quality']['maximum_tested_blank_lane_halfwidth']:.1f} m"
        ax.set_title(label,fontsize=9)
    for ax in list(axes.flat)[len(cases):]:ax.set_visible(False)
    fig.suptitle(title,fontsize=14);fig.savefig(destination,dpi=160);plt.close(fig)
