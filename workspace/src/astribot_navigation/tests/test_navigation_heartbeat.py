"""Exercise the real HTTP handler without ROS or a chassis connection."""
import ast
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from real_navigation.web import server


def test_stale_browser_snapshot_does_not_report_fault_or_renew_old_task():
    owner = 'a'*32
    state = dict(busy=True, task='current', owner=owner)
    node = SimpleNamespace(token='token', state=lambda: state)
    http = server(node, 0)
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def heartbeat(task,who=owner):
        req=urllib.request.Request('http://127.0.0.1:%s/api/heartbeat'%http.server_address[1],
            data=json.dumps(dict(owner=who,task=task)).encode(),
            headers={'X-Session-Token':'token','Content-Type':'application/json'})
        with opener.open(req,timeout=2) as r:return json.load(r)
    try:
        assert heartbeat('previous') == {'superseded': True}
        assert heartbeat('current') == {'queued': True}
        assert heartbeat('current', 'b'*32) == {'superseded': True}
        state['busy'] = False
        assert heartbeat('current') == {'superseded': True}
    finally:
        http.shutdown()
        http.server_close()


def test_transform_tolerance_accepts_recorded_jitter_but_rejects_stale_tf():
    tree=ast.parse((ROOT/'scripts/real_navigation/supervisor.py').read_text())
    expr=next(n.value for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='tf_ok' for t in n.targets) and isinstance(n.value,ast.Compare))
    code=compile(ast.Expression(body=expr),'tf','eval')
    for age,expected in [(.529,True),(.8,True),(.801,False),(-.351,False)]:
        assert eval(code,{'tf_age':age}) is expected
