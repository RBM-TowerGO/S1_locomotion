"""Isolated coordinator tests: no ROS graph or hardware commands."""
import ast
from pathlib import Path
from types import SimpleNamespace as S
import pytest

ROOT=Path(__file__).resolve().parents[1]

def coordinator():
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Web')
    names={'pause_allowed','pause_navigation','tick_data_pause','prepare_navigation'}
    cls=ast.ClassDef(name='Coordinator',bases=[],keywords=[],body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names],decorator_list=[])
    import sys
    sys.path.insert(0,str(ROOT/'scripts'))
    from real_navigation.core import sensor_fresh
    ns={'sensor_fresh':sensor_fresh,'NS':'/towergo/nav/'}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),'web','exec'),ns)
    w=ns['Coordinator']();w.sent=[];w.events=[];w.stops=[];w.starts=[]
    w.send=lambda op,**kw:w.sent.append((op,kw));w.event=w.events.append
    w.stop=lambda *a,**kw:(w.stops.append(a),setattr(w,'data_pause',None))
    w.begin=w.starts.append
    w.count_publishers=lambda t:1
    w.sensors=S(error=None,recovering_until=0,received={k:100 for k in ('front','back','imu','lio')},stamps={k:100 for k in ('front','back','imu','lio')})
    w.quality=S(monitor=S(invalid=False,last_trusted=[0,0,0],state={'status':'trusted'}),recovery=S(status='idle'))
    w.executor_at=100;w.executor_state={'rights':True,'stationary':True,'task':None}
    w.confidence=S(blocked=False);w.amcl_cov=.05;w.amcl_active=True;w.amcl_lifecycle_at=100;w.active_lifecycle=True;w.lifecycle_at=100
    w.goal_attempt=1;w.action_finished=False;w.goal_handle=S(cancel_goal_async=lambda:w.events.append('cancel'))
    w.lease=S(task='original',owner='owner',stop=lambda:w.events.append('lease revoked'))
    w.state={'goal':[1,2,.3],'requested_speed':.05};w.localization={}
    w.params_ok=True
    w.pause_navigation(100,'等待扫描与定位变换恢复')
    return w

def fresh(w,t):
    w.executor_at=w.amcl_lifecycle_at=w.lifecycle_at=t
    for k in w.sensors.received:w.sensors.received[k]=w.sensors.stamps[k]=t

def test_pause_requests_hold_cancels_action_and_preserves_goal():
    w=coordinator()
    assert w.sent==[('cancel_stationary',{})] and 'cancel' in w.events
    assert w.phase=='paused' and w.goal_attempt==2 and w.state['command_velocity']==[0,0,0]
    assert w.state['goal']==[1,2,.3] and not w.starts

def test_resume_requires_two_continuous_seconds_and_old_action_finished():
    w=coordinator()
    w.tick_data_pause(100,100,True)
    assert w.data_pause['good_since'] is None
    w.action_finished=True
    for t,healthy in [(101,True),(102,False),(103,True),(104.99,True)]:
        fresh(w,t);w.tick_data_pause(t,t,healthy)
    assert not w.starts
    fresh(w,105);w.tick_data_pause(105,105,True)
    assert w.phase=='preparing' and w.sent[-1]==('prepare',dict(task='original',speed=.05))
    assert 'lease revoked' not in w.events and w.data_pause is None

@pytest.mark.parametrize('kind',['rights','fault','mismatch','history','publisher','lifecycle'])
def test_other_failures_cancel_without_resume(kind):
    w=coordinator();w.action_finished=True
    if kind=='rights':w.executor_state['rights']=False
    if kind=='fault':w.executor_state['fault']='SDK fault'
    if kind=='mismatch':w.quality.monitor.invalid=True
    if kind=='history':w.sensors.error='history reset'
    if kind=='publisher':w.count_publishers=lambda t:2
    if kind=='lifecycle':w.active_lifecycle=False
    w.tick_data_pause(100,100,True)
    assert w.stops and not w.starts and w.data_pause is None

@pytest.mark.parametrize('field',['stationary','holding','task'])
def test_stationary_and_hold_completion_required(field):
    w=coordinator();w.action_finished=True
    w.executor_state[field]={'stationary':False,'holding':True,'task':'old'}[field]
    for t in [100,103]:
        fresh(w,t);w.tick_data_pause(t,t,True)
    assert not w.starts and w.data_pause['good_since'] is None

def test_long_scan_gap_keeps_original_task_until_data_returns():
    w=coordinator();w.action_finished=True
    for t in [110,150,250]:
        fresh(w,t);w.tick_data_pause(t,t,False)
        assert w.phase=='paused' and not w.stops
    fresh(w,251);w.tick_data_pause(251,251,True)
    fresh(w,253);w.tick_data_pause(253,253,True)
    assert w.phase=='preparing' and w.sent[-1]==('prepare',dict(task='original',speed=.05))

def test_stop_clears_pause_and_invalidates_old_action():
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Web')
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='stop')
    end=next(i+1 for i,n in enumerate(fn.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Attribute) and t.attr=='goal_attempt' for t in n.targets))
    ns={};exec(compile(ast.fix_missing_locations(ast.Module(body=fn.body[:end],type_ignores=[])),'stop-prefix','exec'),{'self':(w:=coordinator())})
    assert w.retreat is None and w.state['recovery'] is None
    assert w.data_pause is None and w.goal_attempt==3

@pytest.mark.parametrize('browser_valid,tf_ok,quality_ok,scan_age,expected',[(True,False,True,.1,'pause'),(True,True,True,1.01,'pause'),(True,True,False,.29,'pause'),(False,False,False,.1,'stop')])
def test_actual_dispatch_prioritizes_manual_lease_loss(browser_valid,tf_ok,quality_ok,scan_age,expected):
    import json
    import sys
    sys.path.insert(0,str(ROOT/'scripts'))
    from real_navigation.core import sensor_fresh
    w=coordinator();w.phase='running';w.data_pause=None
    w.lease.active=True;w.lease.task='task';w.valid=lambda t:browser_valid
    w.state.update(ready=False,ready_checks={'sensors':False})
    w.sensors.received['scan']=100;w.sensors.stamps['scan']=100-scan_age
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Web')
    tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_tick')
    block=next(n for n in tick.body if isinstance(n,ast.If) and ast.unparse(n.test)=='self.lease.active')
    exec(compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'dispatch','exec'),
         dict(self=w,now=100,ros_now=100,localized=tf_ok and quality_ok,execution=True,json=json))
    assert bool(w.stops)==(expected=='stop')
    assert (w.phase=='paused')==(expected=='pause')


def test_data_pause_does_not_reset_retreat_attempt_budget():
    w=coordinator();w.recovery_attempts=1;w.action_finished=True
    fresh(w,100);w.tick_data_pause(100,100,True)
    fresh(w,102.01);w.tick_data_pause(102.01,102.01,True)
    assert w.phase=='preparing' and w.recovery_attempts==1


def test_blocked_reverse_rechecks_the_same_arc_and_restarts_clear_time():
    w=coordinator();w.goal_handle=None;w.action_finished=True
    w.phase='running';w.data_pause=None;w.lease.active=True;w.valid=lambda task:True
    w.cmd=(-.03,-.004,.2);w.cmd_at=w.nav_feedback_at=100.
    w.state['ready']=True;w.executor_state['task']=w.lease.task
    w.config={'collision_time_horizon':1.5}
    samples=iter([False,False,None,True,False,True,True,True]);queries=[]
    def clearance(distance,**twist):
        queries.append((distance,twist))
        return next(samples),'后退路径有障碍'
    w.retreat_clearance=clearance
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Web')
    tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_tick')
    block=next(n for n in tick.body if isinstance(n,ast.If) and ast.unparse(n.test)=='self.lease.active')
    from real_navigation.core import navigation_command
    exec(compile(ast.Module(body=[block],type_ignores=[]),'reverse-dispatch','exec'),
         dict(self=w,now=100,ros_now=100,navigation_command=navigation_command))
    assert w.phase=='paused' and w.sent[-1]==('cancel_stationary',{})
    w.executor_state['task']=None
    for t in (101,130,131,132,140,141.99):
        fresh(w,t);w.tick_data_pause(t,t,True)
        assert w.phase=='paused' and not w.stops
    fresh(w,142);w.tick_data_pause(142,142,True)
    assert queries==[(.045,dict(vx=-.03,vy=-.004,w=.2))]*8
    assert w.sent[-1]==('prepare',dict(task='original',speed=.05))
    assert w.state['goal']==[1,2,.3] and w.phase=='preparing'


def test_stale_executor_status_waits_for_a_fresh_stationary_report():
    w=coordinator();w.action_finished=True
    for t in (101,120):
        fresh(w,t);w.executor_at=t-1
        w.tick_data_pause(t,t,True)
        assert w.phase=='paused' and w.data_pause['good_since'] is None and not w.stops
    fresh(w,121);w.tick_data_pause(121,121,True)
    fresh(w,123);w.tick_data_pause(123,123,True)
    assert w.phase=='preparing'
