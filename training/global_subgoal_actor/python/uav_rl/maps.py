"""APF failure families with controller-independent five-body certificates.

Bamboo is sampled before any route exists. No stem is removed for a route.
Only the five-aircraft launch/arrival neighbourhoods are protected.
"""
import heapq,itertools,json
import numpy as np
from scipy import sparse
from scipy.sparse.csgraph import dijkstra
from scipy.ndimage import distance_transform_edt
from .config import FAMILIES,CONTEXTS
from .io import digest

OFFSETS=np.array([[0,0,0],[-3,2.5,0],[-3,-2.5,0],[-6,5,0],[-6,-5,0]],float)


def box(lo,hi):
    lo,hi=np.asarray(lo,float),np.asarray(hi,float)
    return {'type':'box','center':((lo+hi)/2).tolist(),'halfSize':((hi-lo)/2).tolist()}


def sdf(points,obstacle):
    points=np.asarray(points,float);c=np.asarray(obstacle['center'])
    if obstacle['type']=='sphere':return np.linalg.norm(points-c,axis=-1)-obstacle['r']
    if obstacle['type']=='box':
        q=np.abs(points-c)-np.asarray(obstacle['halfSize'])
        return np.linalg.norm(np.maximum(q,0),axis=-1)+np.minimum(np.max(q,axis=-1),0)
    qr=np.linalg.norm(points[...,:2]-c,axis=-1)-obstacle['r']
    qz=np.abs(points[...,2]-(obstacle['zmin']+obstacle['zmax'])/2)-(obstacle['zmax']-obstacle['zmin'])/2
    return np.hypot(np.maximum(qr,0),np.maximum(qz,0))+np.minimum(np.maximum(qr,qz),0)


def clearance(points,obstacles,bounds=None):
    points=np.asarray(points,float);answer=np.full(points.shape[:-1],np.inf)
    if bounds is not None:answer=np.minimum(np.min(points-np.asarray(bounds[0]),axis=-1),np.min(np.asarray(bounds[1])-points,axis=-1))
    for obstacle in obstacles:answer=np.minimum(answer,sdf(points,obstacle))
    return answer


def segment_ok(a,b,obstacles,bounds,clear=1.2,offsets=OFFSETS,step=.4):
    a,b=np.asarray(a),np.asarray(b);n=max(1,int(np.ceil(np.linalg.norm(b-a)/step)))
    actual=np.linalg.norm(b-a)/n;points=a+(b-a)*np.linspace(0,1,n+1)[:,None]
    points=points[:,None,:]+offsets[None,:,:]
    return bool(np.min(clearance(points,obstacles,bounds))>=clear+actual/2-1e-9)


def graph(case,fleet=True,step=2.):
    upper=np.asarray(case['workspace'][1])
    extruded=bool(case['obstacles']) and abs(case['start'][2]-case['goal'][2])<1e-9 and all(
        (o['type']=='cylinder' and o['zmin']<=0 and o['zmax']>=upper[2]) or
        (o['type']=='box' and o['center'][2]-o['halfSize'][2]<=0 and o['center'][2]+o['halfSize'][2]>=upper[2]) for o in case['obstacles'])
    # A full-height extrusion has exactly the same free XY configurations at
    # every usable altitude; solve that plane instead of duplicate Z layers.
    z=np.asarray([case['start'][2]]) if extruded else np.arange(4,upper[2]-2,step)
    axes=[np.arange(8,upper[0]-6,step),np.arange(8,upper[1]-6,step),z]
    xyz=np.stack(np.meshgrid(*axes,indexing='ij'),-1);shape=xyz.shape[:-1]
    offsets=OFFSETS if fleet else np.zeros((1,3));free=np.ones(shape,bool)
    for offset in offsets:
        # Every 6-neighbour edge is <=step. The endpoint Lipschitz margins
        # certify its full continuous segment, not just its grid vertices.
        free &= clearance(xyz+offset,case['obstacles'],case['workspace'])>=1.2+step/2+1e-8
    ids=np.arange(free.size,dtype=np.int32).reshape(shape);rows=[];cols=[]
    for axis in range(3):
        left=[slice(None)]*3;right=left.copy();left[axis]=slice(None,-1);right[axis]=slice(1,None)
        left,right=tuple(left),tuple(right);valid=free[left]&free[right]
        rows.append(ids[left][valid]);cols.append(ids[right][valid])
    a=np.concatenate(rows);b=np.concatenate(cols);r=np.r_[a,b];c=np.r_[b,a]
    matrix=sparse.csr_matrix((np.full(len(r),step),(r,c)),shape=(free.size,free.size))
    def index(point):return np.ravel_multi_index(tuple(int(np.argmin(np.abs(ax-q))) for ax,q in zip(axes,point)),shape)
    return axes,xyz,free,matrix,index


def witness(case):
    axes,xyz,free,matrix,index=graph(case);start=index(case['start']);goal=index(case['goal'])
    if not free.flat[start] or not free.flat[goal]:return None
    distances,predecessors=dijkstra(matrix,indices=goal,return_predecessors=True)
    if not np.isfinite(distances[start]):return None
    route=[np.asarray(case['start'])];current=start
    while current!=goal:
        current=int(predecessors[current])
        if current<0:return None
        route.append(xyz.reshape(-1,3)[current])
    route.append(np.asarray(case['goal']));points=[]
    for p in route:
        if not points or np.linalg.norm(points[-1]-p)>1e-9:points.append(p)
    # Simplification is checked against the actual solids and all five bodies.
    result=[points[0]];i=0
    while i<len(points)-1:
        j=len(points)-1
        while j>i+1 and not segment_ok(points[i],points[j],case['obstacles'],case['workspace']):j-=1
        if not segment_ok(points[i],points[j],case['obstacles'],case['workspace']):return None
        result.append(points[j]);i=j
    lengths=np.linalg.norm(np.diff(result,axis=0),axis=1);speed=np.minimum(3.,np.sqrt(3.*lengths))
    seconds=3.+np.sum(lengths/np.maximum(speed,1e-9)+speed/3.)
    if seconds>115:return None
    return {'points':np.asarray(result).tolist(),'stop_at_vertices_seconds':float(seconds),'clearance':1.2,'type':'continuous_five_body_translation','controller_used':False}


def geodesic_field(case):
    axes,xyz,free,matrix,index=graph(case,fleet=False);goal=index(case['goal'])
    distances=dijkstra(matrix,indices=goal).reshape(free.shape);valid=np.isfinite(distances)
    if not valid.any():raise ValueError('Goal has no reachable potential grid.')
    # Filling unreachable/inflated cells defines a bounded potential only.
    # It is never a policy input or a collision certificate.
    nearest=distance_transform_edt(~valid,return_distances=False,return_indices=True)
    filled=distances[tuple(nearest)]
    extra=np.sqrt(np.sum((nearest-np.indices(free.shape))**2,axis=0))*2.
    filled=np.minimum(200.,filled+extra).astype(np.float32)
    return {'axes':axes,'values':filled}


def field_values(field,points):
    from .reward_v2 import field_values_linear
    return field_values_linear(field,points)


def protected(obstacle,start,goal):
    # Nothing along a future route is used here.
    for endpoint,required in [(start,2.2),(goal,4.2)]:
        points=np.asarray(endpoint)+OFFSETS
        if np.min(sdf(points,obstacle))<required:return True
    for a,b in [(start,np.asarray(start)+[5,0,0]),(np.asarray(goal)-[5,0,0],goal)]:
        points=np.asarray(a)+(np.asarray(b)-a)*np.linspace(0,1,11)[:,None]
        if np.min(sdf(points[:,None,:]+OFFSETS[None,:,:],obstacle))<1.5:return True
    return False


def bamboo_obstacles(rng,start,goal,count,width=96.,existing=None):
    output=list(existing or []);stems=[]
    for _ in range(10000):
        if len(stems)>=count:break
        x,y=rng.uniform([8,7],[width-7,73]);r=float(rng.uniform(.45,.95))
        if rng.random()<.72:c={'type':'cylinder','center':[float(x),float(y)],'r':r,'zmin':0.,'zmax':30.}
        else:c=box([x-r,y-r,0],[x+r,y+r,30])
        if protected(c,start,goal):continue
        if any(np.linalg.norm(np.asarray(s['center'][:2])-[x,y])<r+(s.get('r') or np.linalg.norm(s['halfSize'][:2]))+.7 for s in stems):continue
        if any(float(sdf(np.array([x,y,10]),o))<r+.3 for o in output):continue
        stems.append(c)
    if len(stems)<count:raise ValueError('Stem placement did not converge.')
    return output+stems


def blank_lane_halfwidth(obstacles,width=96.):
    # A predeclared set of long interior lines, away from permitted endpoint
    # clearings. Circumscribed discs make a wide empty result conservative.
    values=[];a_x=24.;b_x=width-24.
    centers=np.asarray([o['center'][:2] for o in obstacles]);radii=np.asarray([o.get('r') or np.linalg.norm(o['halfSize'][:2]) for o in obstacles])
    for y0 in np.arange(14.,67.,2.):
        for slope in [-.5,-.25,0.,.25,.5]:
            y1=y0+slope*(b_x-a_x)
            if not 14<=y1<=66:continue
            a=np.array([a_x,y0]);b=np.array([b_x,y1]);delta=b-a
            t=np.clip(((centers-a)*delta).sum(1)/np.dot(delta,delta),0,1)
            distances=np.linalg.norm(centers-(a+t[:,None]*delta),axis=1)-radii
            values.append(float(np.min(distances)))
    return max(values,default=0.)


def walls(rng,family,width=96.):
    result=[]
    def wall(x,z,gap):return [box([x,0,0],[x+2.2,80,z-gap/2]),box([x,0,z+gap/2],[x+2.2,80,30])]
    if family in ['single_window','double_window','multi_window','height_chicane']:
        n={'single_window':1,'double_window':2,'multi_window':int(rng.integers(3,6)),'height_chicane':3}[family]
        xs=np.linspace(30,66,n) if n>1 else [rng.uniform(35,43)]
        levels=[float(rng.uniform(17,21))] if n==1 else [float(rng.uniform(7.5,9.5) if i%2 else rng.uniform(19,22)) for i in range(n)]
        if rng.random()<.5:levels=levels[::-1]
        for x,z in zip(xs,levels):result+=wall(float(x),z,float(rng.uniform(9.5,12.)))
    elif family in ['u_trap','c_trap','nested_u','dead_end','fork_trap']:
        front=float(rng.uniform(30,35));back=front+float(rng.uniform(16,22));half=float(rng.uniform(9,13))
        result=[box([front,40-half,0],[back,42-half,30]),box([front,38+half,0],[back,40+half,30])]
        if family=='c_trap':result.append(box([front,40-half,0],[front+2,40+half,30]))
        else:result.append(box([back-2,40-half,0],[back,40+half,30]))
        if family=='nested_u':
            result += [box([front+5,34,0],[back-5,36,30]),box([front+5,44,0],[back-5,46,30]),box([back-7,34,0],[back-5,46,30])]
        if family=='fork_trap':result += [box([front-8,17,0],[front+2,20,30]),box([front-8,17,0],[front-6,30,30])]
    elif family=='narrow_corridor':
        half=float(rng.uniform(9.5,12));result=[box([25,0,0],[69,40-half,30]),box([25,40+half,0],[69,80,30])]
        result.append(box([48,40-half,0],[51,42.3-half,30]))
    elif family in ['maze','staggered_wall']:
        for i,x in enumerate([29,44,59]):
            center=float((25 if i%2 else 55)+rng.uniform(-3,3));gap=float(rng.uniform(18,23))
            result += [box([x,0,0],[x+2,center-gap/2,30]),box([x,center+gap/2,0],[x+2,80,30])]
    elif family=='slalom':
        for i,x in enumerate([30,46,62]):result.append({'type':'cylinder','center':[float(x),float(40+(-1)**i*rng.uniform(3,6))],'r':float(rng.uniform(3.2,5.2)),'zmin':0.,'zmax':30.})
    return result


def geometry(rng,family,settings,split):
    width=96.;modules=[family];start=[14.,40.,10.];goal=[82.,40.,10.]
    if family=='mixed':
        for _ in range(100):
            modules=list(rng.choice(['window','u','bamboo','slalom','sphere','staggered'],size=int(rng.integers(2,4)),replace=False))
            bucket=int(digest(modules)[:8],16)%10
            allowed=settings['topology_train_buckets'] if split=='train' else settings['topology_validation_buckets'] if split=='validation' else settings['topology_test_buckets']
            if bucket in allowed:break
        else:raise ValueError('Topology holdout selection failed.')
        width=120. if len(modules)==3 else 96.;goal=[width-14.,40.,10.];obstacles=[]
        for i,kind in enumerate(modules):
            x=28.+i*24+float(rng.uniform(-1.8,1.8))
            if kind=='window':
                z=float(rng.uniform(18,21));gap=11.;obstacles += [box([x,0,0],[x+2,80,z-gap/2]),box([x,0,z+gap/2],[x+2,80,30])]
            elif kind=='u':
                half=float(rng.uniform(9,12));back=x+float(rng.uniform(13,17));thick=float(rng.uniform(1.5,2.2))
                obstacles += [box([x,40-half,0],[back,40-half+thick,30]),box([x,40+half-thick,0],[back,40+half,30]),box([back-thick,40-half,0],[back,40+half,30])]
            elif kind=='slalom':obstacles.append({'type':'cylinder','center':[x+5,float(rng.uniform(35,45))],'r':float(rng.uniform(3.5,5.)),'zmin':0.,'zmax':30.})
            elif kind=='sphere':obstacles.append({'type':'sphere','center':[x+5,float(rng.uniform(36,44)),float(rng.uniform(8,14))],'r':float(rng.uniform(4,6))})
            elif kind=='staggered':obstacles += [box([x,0,0],[x+2,float(rng.uniform(31,37)),30]),box([x+9,float(rng.uniform(45,51)),0],[x+11,80,30])]
        if 'bamboo' in modules:obstacles=bamboo_obstacles(rng,start,goal,int(rng.integers(35,53)),width,obstacles)
    elif family=='bamboo':obstacles=bamboo_obstacles(rng,start,goal,int(rng.integers(settings['bamboo_count'][0],settings['bamboo_count'][1]+1)),width)
    elif family=='dense_mixed':
        obstacles=[]
        for _ in range(int(rng.integers(40,66))):
            x,y=rng.uniform([9,7],[87,73]);r=float(rng.uniform(.6,1.4));choice=int(rng.integers(3))
            o=box([x-r,y-r,0],[x+r,y+r,rng.uniform(18,30)]) if choice==0 else {'type':'cylinder','center':[float(x),float(y)],'r':r,'zmin':0.,'zmax':float(rng.uniform(22,30))} if choice==1 else {'type':'sphere','center':[float(x),float(y),float(rng.uniform(6,24))],'r':float(rng.uniform(1.8,3.5))}
            if not protected(o,start,goal):obstacles.append(o)
    else:obstacles=walls(rng,family,width)
    if any(protected(o,start,goal) for o in obstacles):raise ValueError('Endpoint protection failed.')
    case={'workspace':[[0.,0.,0.],[width,80.,30.]],'start':start,'goal':goal,'obstacles':obstacles,'terrain_modules':modules}
    if family=='bamboo':
        lane=blank_lane_halfwidth(obstacles,width)
        if lane>settings['bamboo_max_blank_lane_halfwidth']:raise ValueError('An overly wide straight empty lane was found; resample whole forest.')
        case['bamboo_quality']={'route_carving':False,'endpoint_protection_only':True,'stem_count':len(obstacles),
            'maximum_tested_blank_lane_halfwidth':lane,'limit':settings['bamboo_max_blank_lane_halfwidth'],'tested_orientations':5,
            'scope':'Long interior straight-line probes; not a proof against every possible curved gap.'}
    return case


def physical_digest(case):
    value={k:case[k] for k in ['workspace','start','goal']}
    value['obstacles']=sorted(case['obstacles'],key=lambda x:json.dumps(x,sort_keys=True,separators=(',',':')))
    return digest(value)


def generate(cfg,split,number,family=None,stratum_number=None):
    family=family or FAMILIES[number%len(FAMILIES)];seed=cfg['training']['seed']+{'train':100000000,'validation':200000000,'test':300000000}[split]+number
    rng=np.random.default_rng(seed);last=''
    for attempt in range(cfg['dataset']['generation_attempts']):
        try:
            case=geometry(rng,family,cfg['dataset'],split);certificate=witness(case)
            if certificate is None:raise ValueError('No conservative five-body path certificate.')
            case['geometric_witness']=certificate
            for a,b in [(case['start'],np.asarray(case['start'])+[5,0,0]),(np.asarray(case['goal'])-[5,0,0],case['goal'])]:
                if not segment_ok(a,b,case['obstacles'],case['workspace']):raise ValueError('Departure/arrival segment failed.')
            break
        except ValueError as error:last=str(error)
    else:raise RuntimeError(f'Generation exhausted {family} {seed}: {last}')
    mirror=bool(rng.integers(2));swap=bool(rng.integers(2));width=case['workspace'][1][0]
    def transform(point):
        p=list(point)
        if mirror:p[0]=width-p[0]
        if swap:p[0],p[1]=p[1],p[0]
        return p
    for obstacle in case['obstacles']:
        if obstacle['type']=='cylinder':obstacle['center']=transform([*obstacle['center'],0.])[:2]
        else:obstacle['center']=transform(obstacle['center'])
        if obstacle['type']=='box' and swap:obstacle['halfSize'][0],obstacle['halfSize'][1]=obstacle['halfSize'][1],obstacle['halfSize'][0]
    case['start']=transform(case['start']);case['goal']=transform(case['goal'])
    certificate['points']=[transform(p) for p in certificate['points']]
    if swap:case['workspace'][1][0],case['workspace'][1][1]=case['workspace'][1][1],case['workspace'][1][0]
    case['augmentation']={'mirror_x':mirror,'swap_xy':swap,'exact_axis_aligned_geometry':True}
    case.update(schema=6,id=f'{split[:1]}{number:09d}',split=split,group=family,seed=seed,stochastic_seed=seed+700000,
        mission_seconds=120.,vmax=3.,amax=3.,reference_route=certificate['points'],generation_attempts=attempt+1)
    ordinal=number if stratum_number is None else stratum_number
    context=CONTEXTS[(ordinal//len(FAMILIES))%len(CONTEXTS)] if split!='train' else str(rng.choice(CONTEXTS,p=cfg['environment']['context_probabilities']))
    perturbed=bool((ordinal//(len(FAMILIES)*len(CONTEXTS)))%2) if split!='train' else bool(rng.random()<cfg['environment']['perturbed_probability'])
    direction=rng.normal(size=3);direction[2]*=.3;direction/=np.linalg.norm(direction)
    case.update(context=context,noise_profile='perturbed' if perturbed else 'clean',noise_std=float(rng.uniform(.005,.015)) if perturbed else 0.,
        wind_bias=(direction*rng.uniform(.1,.3)).tolist() if perturbed else [0.,0.,0.],wind_intervals_seconds=[[12.,25.],[47.,61.],[80.,94.]],
        random_acceleration_max=float(rng.uniform(.04,.12)) if perturbed else 0.,sensor_delay_steps=int(rng.integers(0,3)) if perturbed else 0,
        ray_noise_std=float(rng.uniform(0,.02)) if perturbed else 0.,ray_dropout_probability=float(rng.uniform(0,.015)) if perturbed else 0.)
    case['task_protocol']='global_subgoal_v1'
    case['geometry_sha256']=physical_digest(case);case['case_sha256']=digest(case)
    return case
