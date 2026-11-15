"""FAST-LIO body-cloud scan path: exact-stamp pose matching, geometry, LIO params."""
import ast,copy,math,sys,time
from pathlib import Path
from types import SimpleNamespace as S,MethodType
from collections import deque
import numpy as np
import pytest
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from real_navigation.pipeline import Metrics
from real_navigation.geometry import Geometry,cloud_xyz,scan_points,matrix
from real_navigation.core import validate_config

CONFIG=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())


def pc2(points,extra_fields=True):
    """PointXYZINormal-like layout (x,y,z,intensity,normal xyz,curvature)."""
    points=np.asarray(points,np.float32).reshape((-1,3))
    step=32 if extra_fields else 12
    data=np.zeros((len(points),step//4),np.float32);data[:,:3]=points
    names=['x','y','z']+(['intensity','normal_x','normal_y','normal_z','curvature'] if extra_fields else [])
    fields=[S(name=n,offset=4*i,datatype=7,count=1) for i,n in enumerate(names)]
    return S(fields=fields,width=len(points),height=1,point_step=step,row_step=step*len(points),
             is_bigendian=False,data=data.tobytes())


def test_cloud_xyz_reads_strided_fields_and_drops_nonfinite():
    out=cloud_xyz(pc2([[1,2,3],[np.nan,0,0],[4,5,6]]))
    assert out.tolist()==[[1,2,3],[4,5,6]]
    assert cloud_xyz(pc2(np.empty((0,3)))).shape==(0,3)


def test_cloud_xyz_rejects_bad_layout():
    m=pc2([[1,2,3]]);m.fields[0].datatype=8
    with pytest.raises(ValueError):cloud_xyz(m)
    m=pc2([[1,2,3]]);m.data=m.data[:-4]
    with pytest.raises(ValueError):cloud_xyz(m)
    m=pc2([[1,2,3]]);m.is_bigendian=True
    with pytest.raises(ValueError):cloud_xyz(m)


def test_body_conversion_matches_per_lidar_ranges():
    """IMU body points converted back equal the front-lidar per-sensor scan."""
    g=Geometry(CONFIG);rng=np.random.default_rng(1)
    a=rng.uniform(-math.pi,math.pi,3000);r=rng.uniform(1,10,3000)
    lidar=np.column_stack([r*np.cos(a),r*np.sin(a),np.full(3000,0.)])
    yaw=.3;pose=matrix([1.,2.,0.],[0,0,math.sin(yaw/2),math.cos(yaw/2)])
    body=lidar@g.imu_lidar[:3,:3].T+g.imu_lidar[:3,3]
    inverse=np.linalg.inv(g.imu_lidar)
    back=body@inverse[:3,:3].T+inverse[:3,3]
    _,expected=g.cloud(lidar,pose);_,actual=g.cloud(back,pose)
    assert np.isfinite(expected).sum()>100
    np.testing.assert_allclose(actual,expected,atol=1e-9,equal_nan=True)


class Time:
    def __init__(self,nanoseconds):self.ns=nanoseconds
class Publisher:
    def __init__(self):self.rows=[]
    def publish(self,m):self.rows.append(m)


def setup(now=10.25):
    tree=ast.parse((ROOT/'scripts/real_navigation/sensors.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Sensors')
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('body','publish_body_scan')]
    for fn in methods:fn.body=[n for n in fn.body if not isinstance(n,(ast.Import,ast.ImportFrom))]
    env=dict(time=time,math=math,np=np,LaserScan=lambda:S(header=S()),FRAME={'front':'front'},
             scan_points=scan_points,cloud_xyz=cloud_xyz)
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods,type_ignores=[])),'production-body','exec'),env)
    g=Geometry(CONFIG);clock=[now]
    n=S(geometry=g,lidar_from_imu=np.linalg.inv(g.imu_lidar),pose_by_ns=deque(maxlen=32),pending_body=None,
        metrics=Metrics(),latest_scan=None,latest_scan_source_stamp=None,latest_scan_stamp_ns=None,
        scan_pub=Publisher(),received={},stamps={},error=None,
        node=S(get_clock=lambda:S(now=lambda:S(nanoseconds=int(clock[0]*1e9)))))
    n.fail=lambda text:setattr(n,'error',text)
    n.seen=lambda key,msg:msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    for name in ('body','publish_body_scan'):setattr(n,name,MethodType(env[name],n))
    return n,clock


def body_msg(ns,points=None):
    if points is None:
        a=np.linspace(-math.pi,math.pi,720,endpoint=False)
        points=np.column_stack([3*np.cos(a),3*np.sin(a),np.zeros(720)])
    m=pc2(points);m.header=S(stamp=S(sec=ns//1000000000,nanosec=ns%1000000000))
    return m


def counters(n):return n.metrics.snapshot()['counters']


def test_body_before_pose_waits_then_publishes_with_exact_stamp():
    n,_=setup();ns=10_200_000_123
    n.body(body_msg(ns))
    assert not n.scan_pub.rows and counters(n)['body_waiting_pose']==1
    n.pose_by_ns.append((ns,np.eye(4)));n.publish_body_scan(10.25)
    assert len(n.scan_pub.rows)==1
    scan=n.scan_pub.rows[0]
    assert scan.header.stamp.sec*1000000000+scan.header.stamp.nanosec==ns
    assert n.latest_scan_stamp_ns==ns and n.stamps['scan']==pytest.approx(10.200000123)
    assert np.isfinite(scan.ranges).sum()>600 and n.pending_body is None


def test_pose_first_publishes_immediately():
    n,_=setup();ns=10_200_000_000
    n.pose_by_ns.append((ns,np.eye(4)));n.body(body_msg(ns))
    assert len(n.scan_pub.rows)==1


def test_never_uses_a_different_stamp_pose():
    n,_=setup()
    n.pose_by_ns.append((10_300_000_000,np.eye(4)))
    n.body(body_msg(10_200_000_000))
    assert not n.scan_pub.rows and n.pending_body is None
    assert counters(n)['body_pose_missing']==1


def test_stale_and_out_of_order_bodies_are_discarded():
    n,clock=setup(now=10.9);ns=10_200_000_000
    n.pose_by_ns.append((ns,np.eye(4)));n.body(body_msg(ns))
    assert not n.scan_pub.rows and counters(n)['body_stale_discarded']==1
    n,clock=setup();n.latest_scan=(10.2,[])
    n.pose_by_ns.append((ns,np.eye(4)));n.body(body_msg(ns))
    assert not n.scan_pub.rows and counters(n)['body_out_of_order_discarded']==1


def test_self_points_inside_footprint_are_filtered():
    n,_=setup();ns=10_200_000_000;g=n.geometry
    # A point 0.2 m ahead of the base origin lies inside the robot body.
    base=np.array([[.2,0.,g.config['slice_height']],[3.,0.,g.config['slice_height']]])
    lidar=(base-g.base_lidar[:3,3])@g.base_lidar[:3,:3]
    body=lidar@g.imu_lidar[:3,:3].T+g.imu_lidar[:3,3]
    n.pose_by_ns.append((ns,np.eye(4)));n.body(body_msg(ns,body))
    ranges=np.asarray(n.scan_pub.rows[0].ranges)
    assert np.isfinite(ranges).sum()==1


def lio_doc():
    from tg_mapping import parameters,topic_names
    base=ROOT.parents[2]/'runtime_ws/install/fast_lio/share/fast_lio/config/mid360.yaml'
    if not base.exists():pytest.skip('FAST-LIO config not installed')
    return parameters(base,topic_names('/towergo/nav/lio'),False)


def test_lio_output_enables_body_cloud_and_checks_extrinsics():
    from real_navigation.session import configure_lio_output
    doc=lio_doc();configure_lio_output(doc,CONFIG)
    p=doc['/**']['ros__parameters']['publish']
    assert p['scan_publish_en'] and p['scan_bodyframe_pub_en'] and not p['dense_publish_en']
    bad=copy.deepcopy(CONFIG);bad['imu_from_lidar'][0][3]+=.01
    with pytest.raises(ValueError):configure_lio_output(lio_doc(),bad)


def test_raw_pair_mode_keeps_cloud_output_off():
    from real_navigation.session import configure_lio_output
    doc=lio_doc();configure_lio_output(doc,dict(CONFIG,scan_input='raw_pair'))
    assert doc['/**']['ros__parameters']['publish']['scan_publish_en'] is False


def test_scan_input_is_validated():
    validate_config(dict(CONFIG,scan_input='raw_pair'))
    with pytest.raises(ValueError):validate_config(dict(CONFIG,scan_input='other'))
