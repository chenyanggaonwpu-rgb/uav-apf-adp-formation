"""Reward-only geometry/trajectory terms; no path labels or safety control changes."""
import itertools
import numpy as np


def field_values_linear(field,points):
    """Continuous multilinear interpolation, including singleton extrusion axes."""
    points=np.asarray(points,dtype=np.float64).reshape(-1,3)
    assert np.isfinite(points).all()
    axes=field['axes'];values=np.asarray(field['values']);lower=[];upper=[];weights=[]
    for dim,axis in enumerate(axes):
        axis=np.asarray(axis,float)
        if len(axis)==1:
            lower.append(np.zeros(len(points),np.int64));upper.append(lower[-1]);weights.append(np.zeros(len(points)))
        else:
            spacing=float(axis[1]-axis[0]);assert spacing>0 and np.allclose(np.diff(axis),spacing)
            x=np.clip((points[:,dim]-axis[0])/spacing,0,len(axis)-1)
            lo=np.floor(x).astype(np.int64)
            lower.append(lo);upper.append(np.minimum(lo+1,len(axis)-1));weights.append(x-lo)
    output=np.zeros(len(points),np.float64)
    for bits in itertools.product([0,1],repeat=3):
        index=tuple(upper[i] if bits[i] else lower[i] for i in range(3))
        weight=np.ones(len(points),np.float64)
        for i in range(3):weight*=weights[i] if bits[i] else 1-weights[i]
        output+=weight*values[index]
    assert np.isfinite(output).all()
    return output


def revisit_flags(points,dt,cell_meters=3.,gap_seconds=8.):
    """Keep reward-v1 event semantics, making the requested x1000 exact.

    Charge the physical entry step into a cell absent for >=8 seconds.
    Continuing to occupy one cell is stagnation, not a new return event.
    """
    points=np.asarray(points,float).reshape(-1,3);n=len(points)-1
    cells=np.floor(points/cell_meters).astype(np.int32);last={};flags=np.zeros(n,bool)
    separation=max(1,round(gap_seconds/dt))
    for k in range(n):
        cell=tuple(cells[k+1]);previous=last.get(cell)
        if previous is not None and k-previous>=separation:flags[k]=True
        last[cell]=k
    return flags


def near_safety_densities(trajectory,history,params,settings):
    """Soft approach costs for all-fleet swept margins, independent of guardian.

    Each category is zero >=1m outside its safety boundary, quadratic inside
    that warning band, and capped at one at/inside the existing hard boundary.
    Safety distances and the native safety layer are not modified.
    """
    trajectory=np.asarray(trajectory,float).reshape(6,5,-1)
    n=trajectory.shape[2]-1;warning=float(settings['near_safety_warning_meters']);assert warning>0
    bounds=np.asarray(params['workspace'],float).reshape(2,3)
    xyz=trajectory[:3]
    boundary=np.minimum(xyz-bounds[0,:,None,None],bounds[1,:,None,None]-xyz).min(axis=(0,1))
    margins={
        'near_obstacle':np.asarray(history['obstacleClearance'],float).reshape(n)-float(params['obsSafe']),
        'near_pair':np.asarray(history['pairDistance'],float).reshape(n)-float(params['uavSafe']),
        'near_workspace':np.minimum(boundary[:-1],boundary[1:])-float(settings['workspace_safety_margin_meters']),
    }
    result={key:np.clip(1-values/warning,0,1)**2 for key,values in margins.items()}
    assert all(v.shape==(n,) and np.isfinite(v).all() for v in result.values())
    return result
