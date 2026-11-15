"""Exact global primitive geometry; no grid aliasing, route labels, or truncation.

MLP input: 16 x 128 local history, current measured leader xyz / SCALE,
then 1289 static map features. Static features are stored once per episode.
All generated maps use <=128 axis-aligned boxes, cylinders and spheres.
Unknown primitive types and overflow are errors, never silently omitted.
"""
import numpy as np

SCALE=np.array([120.,120.,30.],dtype=np.float32)
MAXIMUM_OBSTACLES=128
GLOBAL_SIZE=9+10*MAXIMUM_OBSTACLES
INPUT_SIZE=2048+3+GLOBAL_SIZE


def encode_map(case):
    obstacles=case['obstacles']
    assert len(obstacles)<=MAXIMUM_OBSTACLES,'Map exceeds the exact global encoder capacity; no truncation is allowed.'
    rows=[]
    for o in obstacles:
        kind=o['type'];types=[float(kind==v) for v in ['box','cylinder','sphere']]
        assert sum(types)==1,'Unsupported primitive in global observation.'
        if kind=='box':center=np.asarray(o['center'],float);extent=np.asarray(o['halfSize'],float)
        elif kind=='cylinder':
            center=np.array([*o['center'][:2],(o['zmin']+o['zmax'])/2],float)
            extent=np.array([o['r'],o['r'],(o['zmax']-o['zmin'])/2],float)
        else:center=np.asarray(o['center'],float);extent=np.repeat(float(o['r']),3)
        assert center.shape==(3,) and extent.shape==(3,) and np.all(extent>0)
        rows.append([1.,*types,*(center/SCALE),*(extent/SCALE)])
    # Sort the final float32 values: MATLAB reconstructs box centers from
    # corners, which can differ by a double-precision ulp before this cast.
    # Sorting doubles could permute almost-identical x coordinates by y.
    rows=np.asarray(rows,np.float32).tolist() if rows else []
    # Center xyz, primitive type, then extent xyz determine stable slots.
    rows.sort(key=lambda r:tuple(r[i] for i in [4,5,6,1,2,3,7,8,9]))
    tokens=np.zeros((MAXIMUM_OBSTACLES,10),np.float32)
    if rows:tokens[:len(rows)]=np.asarray(rows,np.float32)
    bounds=np.asarray(case['workspace'],float)
    features=np.concatenate([(bounds/SCALE).reshape(-1),np.asarray(case['goal'],float)/SCALE,tokens.reshape(-1)]).astype(np.float32)
    assert features.shape==(GLOBAL_SIZE,) and np.isfinite(features).all()
    return features


def state_at(episode,index):
    frames=np.maximum(0,int(index)-2*np.arange(15,-1,-1))
    return np.concatenate([episode.observations[frames].reshape(2048),episode.positions[index]/SCALE,episode.global_features]).astype(np.float32)


def decode_geometry(features):
    """Independent inverse for lossless-geometry diagnostics and inspection."""
    features=np.asarray(features,dtype=np.float64);assert features.shape==(GLOBAL_SIZE,)
    tokens=features[9:].reshape(MAXIMUM_OBSTACLES,10);obstacles=[]
    for row in tokens:
        if row[0]==0:
            assert np.all(row==0);continue
        assert row[0]==1 and np.count_nonzero(row[1:4])==1
        kind=['box','cylinder','sphere'][int(np.argmax(row[1:4]))]
        center=row[4:7]*SCALE;extent=row[7:10]*SCALE
        if kind=='box':o={'type':kind,'center':center.tolist(),'halfSize':extent.tolist()}
        elif kind=='cylinder':o={'type':kind,'center':center[:2].tolist(),'r':float(extent[0]),'zmin':float(center[2]-extent[2]),'zmax':float(center[2]+extent[2])}
        else:o={'type':kind,'center':center.tolist(),'r':float(extent[0])}
        obstacles.append(o)
    return {'workspace':(features[:6].reshape(2,3)*SCALE).tolist(),'goal':(features[6:9]*SCALE).tolist(),'obstacles':obstacles}
