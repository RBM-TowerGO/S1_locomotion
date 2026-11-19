"""ROS-side orchestration. Search/quality run off the Web control callback."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .localization import MapMatcher, QualityMonitor, Recovery, UpdateExpectation
from .core import SCAN_MAX_SOURCE_AGE
from .confidence import StaticReassessment


class LocalizationRuntime:
    def __init__(self, web):
        self.web=web
        self.matcher=MapMatcher(web.grid,web.sensors.geometry.base_lidar[:2,3])
        self.monitor=QualityMonitor();self.recovery=Recovery()
        self.expectation=UpdateExpectation(web.config.get('amcl_update_min_d',0.),web.config.get('amcl_update_min_a',0.))
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='localization-quality')
        self.future=None;self.search=None;self.generation=0;self.last_scan=None
        self.packet=None;self.last_submit=0.;self.publisher_check=0.;self.publishers={};self.last_nudge=0.;self.nudges=0
        self.last_nomotion=0.
        self.log=(web.run/'localization_quality.jsonl').open('a',buffering=1)
        from std_srvs.srv import Empty
        self.empty=Empty
        self.nudge=web.create_client(Empty,'/towergo/nav/request_nomotion_update')
        self.nudge_future=None
        self.reassessment=StaticReassessment();self.previous_amcl_odom=None
        self.amcl_log=(web.run/'amcl_diagnostics.jsonl').open('a',buffering=1)

    def reset(self, cancel_recovery=True):
        self.generation+=1;self.monitor.reset();self.expectation.reset();self.last_scan=None;self.packet=None
        self.web.sensors.fused_scans.clear()
        self.nudges=0;self.last_nudge=0.
        self.reassessment.reset();self.previous_amcl_odom=None;self.web.confidence.reset()
        if cancel_recovery:self.recovery.cancel()

    def cancel(self):
        self.generation+=1;self.recovery.cancel();self.recovery.status='canceled';self.reassessment.reset()

    def close(self):
        self.cancel();self.pool.shutdown(wait=True,cancel_futures=True);self.log.close();self.amcl_log.close()

    def compute(self, packet):
        started=time.monotonic()
        packet['metrics']=self.matcher.evaluate(packet['points'],packet['pose'])
        packet['per_sensor']={k:self.matcher.evaluate(v['points'],v['pose']) for k,v in packet['sensors'].items()}
        packet['quality_processing_ms']=(time.monotonic()-started)*1000
        self.log.write(json.dumps(packet,ensure_ascii=False,allow_nan=False)+'\n')
        return packet

    def sample(self, now):
        import rclpy
        from .sensors import FRAME
        from .geometry import scan_points
        from tf2_ros import TransformException
        w=self.web;s=w.sensors
        ros_now=w.get_clock().now().nanoseconds*1e-9
        def lookup(stamp_ns):
            return w.tf.lookup_transform(FRAME['map'],FRAME['base'],rclpy.time.Time(nanoseconds=stamp_ns))
        for stamp_ns,source_stamp,received,cloud in reversed(s.fused_scans):
            stamp=stamp_ns*1e-9
            if self.last_scan is not None and stamp<=self.last_scan:return None
            if self.recovery.status=='verifying' and (
                    w.amcl_stamp<=self.recovery.published_stamp or stamp<self.recovery.published_stamp+.2):return None
            age=ros_now-source_stamp
            if not -.05<=age<=SCAN_MAX_SOURCE_AGE or now-received>=.8:continue
            try:
                transform=lookup(stamp_ns)
            except TransformException:
                continue
            break
        else:return None
        def at(t,points):
            q=t.transform.rotation;p=t.transform.translation
            rot=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            yaw=math.atan2(rot[1,0],rot[0,0]);c,sn=math.cos(yaw),math.sin(yaw)
            # Preserve measured roll/pitch in endpoints; search changes planar pose only.
            pts=(np.asarray(points).reshape((-1,3))@rot.T)[:,:2]@np.array([[c,-sn],[sn,c]])
            matrix=np.eye(4);matrix[:3,:3]=rot;matrix[:3,3]=[p.x,p.y,p.z]
            return dict(pose=[p.x,p.y,yaw],points=pts.tolist(),map_from_base=matrix.tolist())
        origin=s.geometry.base_lidar[:3,3]
        fused=at(transform,np.asarray(cloud).reshape((-1,3))+origin)
        sensors={}
        for key,(ts,ranges) in s.scans.items():
            try:sensors[key]=dict(stamp=ts,**at(lookup(int(round(ts*1e9))),scan_points(ranges)+origin))
            except Exception:pass
        odom=s.geometry.at(stamp)
        return dict(generation=self.generation,stamp=stamp,source_stamp=source_stamp,
                    received_monotonic=received,wall_time=time.time(),source_age_s=age,tf_publishers=self.publishers,
                    stationary=bool(0<=now-w.executor_at<.3 and w.executor_state.get('stationary',False)),
                    odom_from_base=odom.tolist() if odom is not None else None,
                    map_from_odom=(np.asarray(fused['map_from_base'])@np.linalg.inv(odom)).tolist() if odom is not None else None,
                    amcl_stamp=w.amcl_stamp if math.isfinite(w.amcl_stamp) and w.amcl_stamp>0 else None,
                    amcl_covariance=w.amcl_cov if math.isfinite(w.amcl_cov) else None,amcl_confidence=w.confidence.snapshot(),
                    executor=w.executor_state,command=list(w.cmd),sensors=sensors,**fused)

    def tick(self, now):
        w=self.web;r=self.recovery
        if now-self.publisher_check>=1:
            self.publisher_check=now
            self.publishers={t:[dict(name=p.node_name,namespace=p.node_namespace) for p in w.get_publishers_info_by_topic(t)]
                             for t in ('/towergo/nav/tf','/towergo/nav/tf_static','/towergo/nav/amcl_pose')}
        stationary=bool(0<=now-w.executor_at<.3 and w.executor_state.get('stationary',False))
        sensors_ready=w.sensors.healthy()
        if self.future is not None and self.future.done():
            try:
                packet=self.future.result()
                if packet['generation']==self.generation and 0<=now-packet['received_monotonic']<.8:
                    self.packet=packet
                    if w.initial_message is not None:
                        # AMCL converging away from the clicked pose is not
                        # stationary drift: start the drift window only once
                        # initialization has succeeded.
                        self.monitor.invalid=False;self.monitor.stationary.clear()
                    self.monitor.update(packet['stamp'],packet['received_monotonic'],packet['metrics'],packet['pose'],packet['stationary'])
            except Exception as exc:
                self.monitor.state=dict(status='unavailable',reason='定位质量计算失败：'+str(exc),last_trusted_pose=self.monitor.last_trusted)
                self.monitor.last_received=None
            self.future=None
        # Scan/map mismatch is advisory: it is displayed and blocks the next
        # goal submission (until re-initialization clears the latch), but it
        # never stops a running task or starts an automatic pose search.
        if self.monitor.invalid and w.localization.get('status')=='succeeded' and w.initial_message is None:
            w.localization.update(status='untrusted',reason=self.monitor.state.get('reason'))
            w.event('定位质量告警（不停止任务）：'+str(self.monitor.state.get('reason')))
        if self.future is None and now-self.last_submit>=.2:
            try:
                packet=self.sample(now)
                if packet:
                    self.last_scan=packet['stamp'];self.last_submit=now
                    self.future=self.pool.submit(self.compute,packet)
            except Exception:
                pass # Freshness expires if the exact-time transform remains unavailable.
        quality=dict(self.monitor.state)
        if self.monitor.last_received is None or now-self.monitor.last_received>=.8:
            quality.update(status='unavailable',reason='缺少新鲜的扫描时刻地图匹配结果')
        quality['recovery']=r.state()
        w.state['localization_quality']=quality
        w.state['last_trusted_pose']=self.monitor.last_trusted
        # Initialization only: AMCL does not refine a stationary filter on its
        # own, so request at most three no-motion updates after initialpose.
        if w.initial_message is not None and stationary and sensors_ready:
            if self.nudges<3 and now-self.last_nudge>=1 and self.nudge.service_is_ready() and (self.nudge_future is None or self.nudge_future.done()):
                self.nudge_future=self.nudge.call_async(self.empty.Request());self.nudges+=1;self.last_nudge=now
        return self.monitor.usable(now)

    def observe_amcl(self, stamp, covariance):
        pose=self.web.sensors.geometry.at(stamp)
        odom=None;delta=None
        if pose is not None:
            odom=[float(pose[0,3]),float(pose[1,3]),math.atan2(pose[1,0],pose[0,0])]
            self.expectation.observe(stamp,odom)
            if self.previous_amcl_odom is not None:
                previous_stamp,previous=self.previous_amcl_odom
                dx,dy=odom[0]-previous[0],odom[1]-previous[1]
                yaw=math.atan2(math.sin(odom[2]-previous[2]),math.cos(odom[2]-previous[2]))
                c,s=math.cos(previous[2]),math.sin(previous[2])
                delta=dict(previous_stamp=previous_stamp,dt=stamp-previous_stamp,
                           x_m=c*dx+s*dy,y_m=-s*dx+c*dy,translation_m=math.hypot(dx,dy),yaw_rad=yaw)
            self.previous_amcl_odom=(stamp,odom)
        else:self.previous_amcl_odom=None
        def finite(v):return float(v) if math.isfinite(v) else None
        w=self.web
        self.amcl_log.write(json.dumps(dict(wall_time=time.time(),received_monotonic=time.monotonic(),
            stamp=stamp,source_age_s=w.get_clock().now().nanoseconds*1e-9-stamp,
            localization_id=w.localization_id,covariance=[finite(v) for v in covariance],
            sigma_x_m=math.sqrt(covariance[0]) if math.isfinite(covariance[0]) and covariance[0]>=0 else None,
            sigma_y_m=math.sqrt(covariance[7]) if math.isfinite(covariance[7]) and covariance[7]>=0 else None,
            sigma_yaw_deg=math.degrees(math.sqrt(covariance[35])) if math.isfinite(covariance[35]) and covariance[35]>=0 else None,
            odom=odom,odom_delta=delta,executor=w.executor_state,command=list(w.cmd)),allow_nan=False)+'\n')

    def amcl_is_current(self, now):
        w=self.web
        if self.expectation.distance<=0 or self.expectation.yaw<=0:
            from .core import localization_fresh
            return localization_fresh(now-w.amcl_at,w.get_clock().now().nanoseconds*1e-9-w.amcl_stamp)
        latest=w.sensors.last_pose
        pose=latest[1] if latest is not None else None
        odom=[pose[0,3],pose[1,3],math.atan2(pose[1,0],pose[0,0])] if pose is not None else None
        return self.expectation.valid(now,odom)
