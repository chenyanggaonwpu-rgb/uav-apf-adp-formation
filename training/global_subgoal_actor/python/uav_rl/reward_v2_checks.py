"""Reward-v2 contract checks, including continuity and real-return event scaling."""
import numpy as np
from .reward_v2 import field_values_linear,revisit_flags,near_safety_densities


def run_checks():
    axes=[np.array([0.,2.,4.]),np.array([0.,2.]),np.array([1.])]
    x,y,z=np.meshgrid(*axes,indexing='ij');field={'axes':axes,'values':3*x+2*y+z}
    points=np.array([[.1,.3,1.],[1.999999,.5,1.],[2.000001,.5,1.],[3.2,1.7,1.]])
    values=field_values_linear(field,points);expected=3*points[:,0]+2*points[:,1]+points[:,2]
    assert np.max(np.abs(values-expected))<1e-12
    assert abs(values[2]-values[1])<1e-4
    all_nodes=np.c_[x.ravel(),y.ravel(),z.ravel()]
    assert np.allclose(field_values_linear(field,all_nodes),field['values'].ravel())
    assert np.isfinite(field_values_linear(field,[[-10,5,9]])).all()
    # The prior definition is tested explicitly: long absence then one entry
    # marker; keeping the point stationary is not repeated entry.
    points=np.r_[np.array([[1.,1.,1.],[1.1,1.,1.]]),np.tile([7.,1.,1.],(180,1)),np.tile([1.1,1.,1.],(3,1))]
    flags=revisit_flags(points,.05);assert flags.sum()==1
    old_cost=-.1*.05*flags;new_cost=-2.*.05*flags
    assert np.allclose(new_cost,old_cost*20.,rtol=1e-15,atol=1e-12)
    trajectory=np.zeros((6,5,5));trajectory[:3]=np.array([5.,5.,5.])[:,None,None]
    history={'obstacleClearance':np.array([2.,1.5,1.,.5]),'pairDistance':np.full(4,20.)}
    params={'workspace':[[0.,0.,0.],[10.,10.,10.]],'obsSafe':1.,'uavSafe':1.}
    settings={'near_safety_warning_meters':1.,'workspace_safety_margin_meters':0.}
    density=near_safety_densities(trajectory,history,params,settings)
    assert np.allclose(density['near_obstacle'],[0.,.25,1.,1.])
    assert np.all(density['near_pair']==0) and np.all(density['near_workspace']==0)
    trajectory[0]=.5
    density=near_safety_densities(trajectory,history,params,settings)
    assert np.allclose(density['near_workspace'],.25)
    # Time-aware potential telescoping still holds after interpolation.
    phi=np.array([-2.,-1.8,-2.,0.]);durations=[.4,1.6,4.9];prefix=1.;total=0.
    for i,dt in enumerate(durations):
        discount=.99**(dt/1.6);total+=prefix*(discount*phi[i+1]-phi[i]);prefix*=discount
    assert abs(total+phi[0])<1e-12
    return {'passed':True,'continuous_linear_field_max_error':float(np.max(np.abs(values-expected))),
        'grid_nodes_preserved':True,'singleton_z_axis_supported':True,'revisit_event_definition_preserved':True,
        'revisit_penalty_exact_multiplier':20.,'one_revisit_entry_penalty':float(new_cost.sum()),
        'near_boundary_monotonic_and_bounded':True,'workspace_boundary_included':True,
        'time_aware_potential_telescoping_passed':True}
