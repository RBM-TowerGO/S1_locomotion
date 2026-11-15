from pathlib import Path
import sys
import yaml
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from real_navigation.session import nav_params

@pytest.mark.parametrize('replay',[False,True])
def test_both_costmaps_use_only_saved_map(replay):
    config=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
    params=nav_params(config,Path('/tmp/unchanged-map.yaml'),replay)['towergo']['nav']
    for name in ('global_costmap','local_costmap'):
        p=params[name][name]['ros__parameters']
        assert p['plugins']==['static_layer','inflation_layer']
        assert 'obstacle_layer' not in p and 'voxel_layer' not in p
        assert p['static_layer']['map_topic']=='/towergo/nav/map'
        assert p['inflation_layer']['inflation_radius']==.75
        assert p['footprint_padding']==config['footprint_padding']
        assert p['track_unknown_space']
    assert params['controller_server']['ros__parameters']['FollowPath']['use_collision_detection']
    assert params['map_server']['ros__parameters']['yaml_filename']=='/tmp/unchanged-map.yaml'
    assert params['amcl']['ros__parameters']['scan_topic']=='/towergo/nav/scan'


def test_turn_profile_uses_configured_turning_radius_primitives():
    import json
    config=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
    radius=config['planning_turning_radius']
    params=nav_params(config,Path('/tmp/map.yaml'),False)['towergo']['nav']
    ps=params['planner_server']['ros__parameters']
    fallback=params['inplace']['planner_server']['ros__parameters']
    # Smac lattice tables are process-wide static: one lattice per process.
    assert ps['planner_plugins']==['GridBased'] and 'InPlaceTurn' not in ps
    assert fallback['planner_plugins']==['InPlaceTurn'] and 'GridBased' not in fallback
    assert params['inplace']['global_costmap']==params['global_costmap']
    assert set(map(tuple,config['footprint']))==set(map(tuple,config['self_filter']))
    for planner,model in ((ps['GridBased'],'ackermann'),(fallback['InPlaceTurn'],'diff')):
        assert planner['plugin']=='astribot_nav_bringup/ExactGoalLattice'
        assert '/%sm_turning_radius/%s/'%(format(radius,'g'),model) in planner['lattice_filepath']
        assert planner['prefer_straight_route'] is False and planner['smooth_path'] is False
        assert planner['allow_reverse_expansion'] is False and planner['allow_unknown'] is False
        path=Path(planner['lattice_filepath'])
        if path.exists():
            metadata=json.loads(path.read_text())['lattice_metadata']
            assert metadata['turning_radius']==radius and metadata['motion_model']==model


def test_in_place_turn_planner_is_only_a_fallback_after_forward_arcs():
    import xml.etree.ElementTree as ET
    tree=ET.parse(ROOT/'scripts/real_navigation/navigation.xml').getroot()
    rate=tree.find('.//RateController')
    fallback=rate.find('Fallback')
    assert fallback is not None and len(list(rate))==1
    assert [n.get('planner_id') for n in fallback]==['GridBased','InPlaceTurn']
    assert all(n.tag=='ComputePathToPose' and n.get('path')=='{path}' for n in fallback)
    assert fallback[0].get('server_name') is None
    assert fallback[1].get('server_name')=='/towergo/nav/inplace/compute_path_to_pose'


def test_launch_runs_in_place_planner_in_separate_process():
    from launch import LaunchContext
    from real_navigation.launch import nodes
    context=LaunchContext();context.launch_configurations['params']='/tmp/nav2.yaml'
    found=[(n.node_package,n.node_executable,n._Node__node_name,n._Node__node_namespace) for n in nodes(context)]
    assert ('nav2_planner','planner_server','planner_server','/towergo/nav') in found
    assert ('nav2_planner','planner_server','planner_server','/towergo/nav/inplace') in found
    assert ('nav2_lifecycle_manager','lifecycle_manager','lifecycle_manager_inplace','/towergo/nav/inplace') in found


def test_start_inside_enlarged_clearance_is_rejected_before_executor_prepare():
    import ast
    from types import SimpleNamespace as S
    import numpy as np
    from real_navigation.core import speed,controller_parameters
    from real_navigation.geometry import safe_pose
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Web')
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='begin')
    ns=dict(speed=speed,controller_parameters=controller_parameters,safe_pose=safe_pose)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'begin','exec'),ns)
    config=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
    grid=S(origin=np.array([0.,0.,0.]),rotation=np.eye(2),resolution=.05,width=200,height=200,free=np.ones((200,200),bool))
    grid.free[100,100]=False
    w=S(mode='hardware',phase='idle',state=dict(ready=True,initialized=True,pose=[5.,5.,0.]),config=config,grid=grid,values=lambda obj:[8.,8.,0.])
    with pytest.raises(ValueError,match='当前位置不满足'):ns['begin'](w,dict(speed=.05))


def test_inflation_covers_outer_padded_corners_for_smac_fast_collision_check():
    import math
    config=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
    padding=config['footprint_padding']
    outer=max(math.hypot(abs(x)+padding,abs(y)+padding) for x,y in config['footprint'])
    assert config['inflation_radius']>=outer+math.sqrt(2)*.05


def test_small_saved_inflation_is_raised_to_cover_padded_corners():
    import math
    from real_navigation.core import costmap_inflation_radius
    config=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
    config['inflation_radius']=.2
    effective=costmap_inflation_radius(config)
    outer=max(math.hypot(abs(x)+config['footprint_padding'],abs(y)+config['footprint_padding']) for x,y in config['footprint'])
    assert effective>=outer+math.sqrt(2)*.05
    p=nav_params(config,Path('/tmp/map.yaml'),False)['towergo']['nav']
    for key in ['global_costmap','local_costmap']:
        assert p[key][key]['ros__parameters']['inflation_layer']['inflation_radius']==effective
