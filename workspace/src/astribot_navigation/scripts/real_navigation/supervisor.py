"""Sensor bridge, localization and task supervision for a headless navigation session."""
import json
import math
import time
from pathlib import Path
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, FollowPath
from rcl_interfaces.srv import SetParametersAtomically
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from astribot_nav_interfaces.srv import NavigationCommand
from tf2_ros import Buffer,TransformListener
from astribot_nav_sim.grid_world import GridWorld
from .geometry import safe_pose
from .localization_runtime import LocalizationRuntime
from .confidence import Confidence
from .core import RouteProgress, GOAL_XY, GOAL_YAW, at_goal, relative_goal, Lease, speed, controller_parameters, Convergence, SCAN_MAX_SOURCE_AGE, BROWSER_LEASE_SECONDS, LOCALIZATION_MAX_SOURCE_AGE, localization_fresh, nav2_failure_reason, costmap_inflation_radius
from .sensors import Sensors,NS,FRAME


class NavigationSupervisor(Node):
    def __init__(self,config,filename,mode,run):
        super().__init__('navigation_supervisor',namespace=NS.rstrip('/'))
        self.config,self.mode,self.run=config,mode,Path(run)
        self.grid=GridWorld(filename)
        self.lease=Lease()
        self.command_service=self.create_service(NavigationCommand, NS+'command', self.handle_command)
        self.state_pub=self.create_publisher(String, NS+'task_state', qos_profile_sensor_data)
        self.sensors=Sensors(self,config);self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.action=ActionClient(self,NavigateToPose,NS+'navigate_to_pose')
        self.follow=ActionClient(self,FollowPath,NS+'follow_path')
        self.parameters=self.create_client(SetParametersAtomically,NS+'controller_server/set_parameters_atomically')
        self.lifecycle=self.create_client(GetState,NS+'bt_navigator/get_state')
        self.amcl_lifecycle=self.create_client(GetState,NS+'amcl/get_state')
        self.amcl_active=False;self.amcl_lifecycle_at=-1e20;self.amcl_lifecycle_pending=False
        self.initial=self.create_publisher(PoseWithCovarianceStamped,NS+'initialpose',1)
        self.control=self.create_publisher(String,NS+'executor_control',10)
        self.create_subscription(String,NS+'executor_status',self.executor_status,1)
        self.create_subscription(PoseWithCovarianceStamped,NS+'amcl_pose',self.amcl,10)
        self.create_subscription(Twist,NS+'cmd_vel_nav',self.command,1)
        self.create_subscription(String,NS+'route_execution',self.route_execution,1)
        self.sequence=0;self.goal_handle=None;self.executor_state={};self.executor_at=-1e20
        self.cmd=(0.,0.,0.);self.cmd_at=-1e20;self.nav_feedback_at=-1e20;self.amcl_at=-1e20;self.amcl_cov=math.inf
        self.amcl_stamp=-1e20;self.initial_stamp=math.inf;self.initial_tf_ns=None;self.initial_tf_error=None
        self.active_lifecycle=False;self.lifecycle_at=-1e20;self.lifecycle_pending=False
        self.goal_attempt=0
        self.tf_paused=False
        self.tf_pause_attempt=None
        self.route_progress=None
        self.relative_odom_goal=None;self.tolerances=(GOAL_XY,GOAL_YAW)
        self.phase='idle';self.phase_at=0.;self.params_ok=False;self.goal_sent=False
        self.parameter_request=None
        self.initial_message=None;self.initial_at=0.;self.initial_last=0.
        self.localization_id=0;self.localization=dict(id=0,status='idle',reason=None)
        self.convergence=Convergence();self.confidence=Confidence()
        self.still_since=None;self.result_pending=None;self.pending_goal=None
        self.state=dict(status='真机观察模式：等待传感器' if mode=='observe' else '真机：等待传感器与执行器',
                        mode=mode,replay=bool(self.get_parameter('use_sim_time').value),ready=False,initialized=False,busy=False,error=None,pose=None,start=None,goal=None,
                        velocity=[0.,0.,0.],command_velocity=[0.,0.,0.],requested_speed=.15,
                        distance_remaining=None,diagnostics=[],task=None,owner=None,result=None,
                        planning_turning_radius=None,localization_trusted=False,localization_warning=None,inflation_radius=costmap_inflation_radius(config),navigation_style='omni',obstacle_source='static_map_and_lidar',live_obstacle_avoidance=True,footprint=config['footprint'],max_speed=config['max_speed'],scan_source_timeout=SCAN_MAX_SOURCE_AGE,localization_source_timeout=LOCALIZATION_MAX_SOURCE_AGE,transform_source_timeout=.8,browser_timeout=BROWSER_LEASE_SECONDS,collision_time_horizon=config.get('collision_time_horizon',1.5),footprint_padding=config['footprint_padding'])
        self.state.update(session=self.run.name, map=self.grid.metadata())
        self.log=(self.run/'navigation.jsonl').open('a',buffering=1)
        self.sensor_epoch=0
        self.last_sample=0.;self.last_lifecycle=0.;self.last_metrics=0.
        self.quality=LocalizationRuntime(self)
        self.create_timer(.05,self._tick)

    def event(self,text):
        self.state['diagnostics']=(self.state['diagnostics']+[text])[-40:]
        self.log.write(json.dumps(dict(time=time.time(),event=text,task=self.state['task'],ready_checks=self.state.get('ready_checks'),sensor_source_stamps=dict(self.sensors.stamps),executor=dict(self.executor_state)),ensure_ascii=False)+'\n')

    def send(self,op,**kwargs):
        self.sequence+=1
        self.control.publish(String(data=json.dumps(dict(op=op,sequence=self.sequence,sent=time.monotonic(),**kwargs))))

    def executor_status(self,msg):
        try:
            data=json.loads(msg.data)
            if 0<=time.monotonic()-data['sent']<.2:
                self.executor_state=data;self.executor_at=time.monotonic()
        except (ValueError,KeyError,TypeError):pass

    def amcl(self,msg):
        now=self.get_clock().now().nanoseconds*1e-9
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.sensors.metrics.add('amcl_source_age_s',now-stamp)
        if msg.header.frame_id!=FRAME['map'] or not localization_fresh(0.,now-stamp) or stamp<=self.amcl_stamp:return
        if stamp<=self.initial_stamp:return
        self.amcl_stamp=stamp;self.amcl_at=time.monotonic()
        if hasattr(self,'quality'):self.quality.observe_amcl(stamp,msg.pose.covariance)
        c=msg.pose.covariance
        self.confidence.observe(c)
        self.amcl_cov=max(c[0],c[7],c[35]) if all(math.isfinite(c[i]) and c[i]>=0 for i in (0,7,35)) else math.inf

    def route_execution(self,msg):
        try:self.state['route_execution']=json.loads(msg.data)
        except (ValueError,TypeError):pass

    def command(self,msg):
        # Display only: the executor subscribes to Nav2's cmd_vel directly.
        self.cmd=(msg.linear.x,msg.linear.y,msg.angular.z);self.cmd_at=time.monotonic()

    def valid(self,task):
        # Goals belong to the session after submission.
        return self.lease.active and task==self.lease.task

    def stop(self,reason='用户取消',result='canceled',recoverable=False):
        self.goal_attempt=getattr(self,'goal_attempt',0)+1
        if result=='canceled' and hasattr(self,'quality'):self.quality.cancel()
        idle_cancel=(result=='canceled' and self.phase=='idle' and not self.lease.active and not self.state['busy'])
        task=self.lease.stop()
        self.tf_paused=False
        self.send('stop' if result=='fault' and not recoverable else 'cancel_stationary')
        self.state['warning']=reason if recoverable else None
        if self.goal_handle:
            self.goal_handle.cancel_goal_async();self.goal_handle=None
        self.params_ok=False;self.cmd=(0.,0.,0.);self.cmd_at=-1e20
        self.initial_message=None
        if hasattr(self,'localization') and self.localization['status']=='pending':
            self.localization.update(status='canceled',reason=reason)
        self.phase='stopping';self.phase_at=time.monotonic();self.still_since=None
        self.result_pending=result;self.state.update(status='正在停车：'+reason,command_velocity=[0.,0.,0.],error=reason if result=='fault' and not recoverable else None)
        if idle_cancel:
            self.phase='idle'
            self.state.update(busy=False,result=result,status='已取消 · 当前没有活动导航任务',error=None)
        self.event(reason)
        # Task remains visible for audit, but the lease is irrevocably invalidated.

    def localization_advisory(self):
        """Why the advisory localization checks are not passing (display only)."""
        if self.confidence.blocked:
            return '定位置信度：'+str(self.confidence.details.get('reason') or '等待稳定确认')
        q=self.state.get('localization_quality') or {}
        if q.get('status')!='trusted':
            return '扫描与地图匹配：'+str(q.get('reason') or q.get('status') or '等待结果')
        return 'AMCL 未按运动门槛输出新结果'

    def localize(self,obj):
        # A manual request replaces any automatic recovery transaction.
        if hasattr(self,'quality') and self.quality.recovery.status not in ('idle','succeeded'):
            self.quality.reset();self.initial_message=None
        reasons=self.localization_reasons()
        if reasons:raise ValueError('暂不能初始化：'+'；'.join(reasons))
        if self.initial_message is not None:raise ValueError('已有定位初始化正在进行，请等待结果或取消')
        if self.state['busy'] or self.phase=='stopping':raise ValueError('请等待停车完成')
        values=self.values(obj,check_footprint=False)
        self.lease.claim(obj['owner'],time.monotonic())
        if hasattr(self,'quality'):self.quality.reset()
        msg=PoseWithCovarianceStamped();msg.header.frame_id=FRAME['map']
        msg.pose.pose.position.x,msg.pose.pose.position.y=values[:2]
        msg.pose.pose.orientation.z=math.sin(values[2]/2);msg.pose.pose.orientation.w=math.cos(values[2]/2)
        msg.pose.covariance[0]=msg.pose.covariance[7]=.25;msg.pose.covariance[35]=.07
        self.state.update(initialized=False,error=None,start=values,status='正在初始化定位；机器人保持静止')
        self.initial_message=msg;self.initial_at=time.monotonic();self.initial_last=0.
        self.convergence.reset()
        self.initial_stamp=self.initial_tf_ns*1e-9
        self.amcl_at=-1e20;self.amcl_cov=math.inf
        if hasattr(self,'confidence'):self.confidence.reset()
        self.initial_message.header.stamp=rclpy.time.Time(nanoseconds=self.initial_tf_ns).to_msg()
        self.localization_id+=1
        self.localization=dict(id=self.localization_id,status='pending',reason=None,source_stamp=self.initial_stamp)
        self.initial.publish(self.initial_message)
        self.initial_last=time.monotonic()
        self.event('已提交一次初始位姿，等待 AMCL 修正')

    def refresh_initialization_transform(self):
        # Confirm a measured scan time once, retaining that exact reference
        # while this node's TF subscription catches up with the next scan.
        stamp=getattr(self.sensors,'latest_scan_stamp_ns',None)
        if stamp is not None and stamp!=self.initial_tf_ns:
            try:
                self.tf.lookup_transform(FRAME['odom'],FRAME['front'],rclpy.time.Time(nanoseconds=stamp))
                self.initial_tf_ns=stamp;self.initial_tf_error=None
            except Exception as exc:self.initial_tf_error=str(exc)
        now=self.get_clock().now().nanoseconds
        age=(now-self.initial_tf_ns)*1e-9 if self.initial_tf_ns is not None else None
        self.state['initialization_tf_age']=round(age,3) if age is not None else None
        return age is not None and -.05<=age<=.5

    def localization_reasons(self):
        reasons=[];now=time.monotonic()
        if self.sensors.error:reasons.append(self.sensors.error)
        if not self.sensors.healthy():reasons.append('双雷达、IMU、里程计或扫描未就绪/已过期')
        if not self.amcl_active or now-self.amcl_lifecycle_at>=3:
            # Lifecycle bring-up is all-or-nothing: any Nav2 node failing to
            # configure leaves AMCL inactive. The cause is in nav2.log.
            reasons.append('AMCL 尚未激活（启动超过 30 秒仍如此时，查看 %s 中的 Failed/error）'%(self.run/'nav2.log'))
        if self.initial.get_subscription_count()<1:reasons.append('AMCL 初始位姿订阅尚未连接')
        scan=self.sensors.latest_scan
        if scan is None:reasons.append('缺少可用扫描')
        elif not self.refresh_initialization_transform():
            reasons.append('缺少新鲜扫描时刻的里程计变换：'+str(self.initial_tf_error or '等待对应 TF'))
        return reasons

    def finish_localization(self,status,reason):
        self.initial_message=None;self.localization.update(status=status,reason=reason)
        self.state.update(initialized=status=='succeeded',error=None if status=='succeeded' else reason,status=reason)
        self.event(reason)

    def poll_amcl_lifecycle(self):
        if self.amcl_lifecycle_pending or not self.amcl_lifecycle.service_is_ready():return
        self.amcl_lifecycle_pending=True
        def reply(f):
            self.amcl_lifecycle_pending=False
            try:self.amcl_active=f.result().current_state.id==3;self.amcl_lifecycle_at=time.monotonic()
            except Exception:self.amcl_active=False
        self.amcl_lifecycle.call_async(GetState.Request()).add_done_callback(reply)


    def values(self,obj,check_footprint=True):
        values=[float(obj[k]) for k in ('x','y','yaw')]
        if not all(math.isfinite(v) for v in values):raise ValueError('选点必须为有限坐标')
        cell=self.grid.cells([values[:2]])[0];x,y=cell
        if not 0<=x<self.grid.width or not 0<=y<self.grid.height or not self.grid.free[y,x]:
            raise ValueError('选点位于障碍、未知区域或地图之外')
        if check_footprint and not safe_pose(self.grid,values,self.config['footprint'],self.config['footprint_padding']):
            raise ValueError('当前朝向下整机轮廓与地图障碍或未知区域相交')
        return values

    def odom_pose(self):
        matrix=self.sensors.last_pose[1]
        return [matrix[0,3],matrix[1,3],math.atan2(matrix[1,0],matrix[0,0])]

    def reached(self):
        return (at_goal(self.state['pose'],self.state['goal'],self.tolerances)
                and (self.relative_odom_goal is None or
                     at_goal(self.odom_pose(),self.relative_odom_goal,self.tolerances)))

    def begin(self,obj,relative=False):
        if self.mode not in ('hardware','mock'):raise ValueError('观察模式不发送运动任务；需先完成现场验证')
        if not self.state['ready'] or not self.state['initialized'] or self.phase!='idle':raise ValueError('定位、执行器或任务尚未就绪')
        # Scan-match quality and covariance are checked once, here. During a
        # running task they are shown as warnings and never stop the robot.
        if not self.state.get('localization_trusted'):
            raise ValueError('定位质量未通过，暂不提交目标：'+self.localization_advisory()+'；持续不通过请重新初始化')
        odom_goal=None
        tolerances=(GOAL_XY,GOAL_YAW)
        if relative:
            dx,dy,turn=(float(obj[k]) for k in ('dx','dy','dyaw'))
            values=relative_goal(self.state['pose'],dx,dy,turn)
            obj=dict(obj,**dict(zip(('x','y','yaw'),values)))
            odom_goal=relative_goal(self.odom_pose(),dx,dy,turn)
            distance=math.hypot(dx,dy)
            tolerances=(min(GOAL_XY,distance/4) if distance else GOAL_XY,
                        min(GOAL_YAW,abs(turn)/4) if turn else GOAL_YAW)
        value=speed(obj['speed']);values=self.values(obj)
        if self.state.get('pose') is None or not safe_pose(self.grid,self.state['pose'],self.config['footprint'],self.config['footprint_padding']):
            raise ValueError('当前位置不满足加大后的机身避障余量；请核对定位与现场位置，在宽敞处重新初始化，勿用修改定位代替实际移车')
        if value>self.config['max_speed']:raise ValueError('超过配置的导航最高速度')
        if not self.parameters.service_is_ready():raise ValueError('控制器参数服务尚未就绪')
        task=self.lease.begin(obj['owner'],time.monotonic())
        self.route_progress=None if relative else RouteProgress(value)
        self.relative_odom_goal=odom_goal;self.tolerances=tolerances
        if hasattr(self,'quality') and self.quality.recovery.status=='succeeded':self.quality.recovery.cancel()
        self.state['warning']=None
        self.state.update(route_execution=None,task=task,owner=obj['owner'],busy=True,goal=values,requested_speed=value,result=None,error=None,status='正在确认导航速度')
        self.params_ok=False
        try:
            self.prepare_navigation(time.monotonic())
        except Exception as exc:
            self.stop(str(exc), 'failed', recoverable=True)
            raise

    def prepare_navigation(self,now):
        task=self.lease.task;value=self.state['requested_speed']
        self.phase='preparing';self.phase_at=now;self.goal_sent=False
        self.cmd=(0.,0.,0.);self.cmd_at=-1e20
        if self.params_ok:return
        if self.parameter_request is not None:
            previous=self.parameter_request;self.parameter_request=None
            previous.cancel()
        params=controller_parameters(value,self.tolerances)
        params.update({'FollowPath.route_task':task,'FollowPath.route_goal_xy':list(self.state['goal'][:2])})
        request=SetParametersAtomically.Request();request.parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in params.items()]
        self.parameter_request=future=self.parameters.call_async(request)
        def applied(f):
            if self.parameter_request is not f or not self.valid(task):return
            self.parameter_request=None
            try:
                result=f.result().result
                if not result.successful:raise RuntimeError(result.reason)
                self.params_ok=True
            except Exception as exc:self.stop('控制器限速失败：'+str(exc),'fault')
        future.add_done_callback(applied)

    def submit_goal(self):
        task=self.lease.task;values=self.state['goal']
        nav_log=self.run/'nav2.log'
        log_offset=nav_log.stat().st_size if nav_log.exists() else 0
        self.goal_attempt+=1;attempt=self.goal_attempt
        def current():return self.valid(task) and self.goal_attempt==attempt
        def stamped(values):
            pose=PoseStamped();pose.header.frame_id=FRAME['map'];pose.header.stamp=self.get_clock().now().to_msg()
            pose.pose.position.x,pose.pose.position.y=values[:2]
            pose.pose.orientation.z=math.sin(values[2]/2);pose.pose.orientation.w=math.cos(values[2]/2)
            return pose
        direct=self.relative_odom_goal is not None
        if direct:
            # A nudge follows this single segment and cannot replan into a detour.
            msg=FollowPath.Goal();msg.controller_id='Nudge';msg.goal_checker_id='goal_checker'
            msg.path.poses=[stamped(self.state['pose']),stamped(values)]
            msg.path.header=msg.path.poses[0].header
            client=self.follow
        else:
            msg=NavigateToPose.Goal();msg.pose=stamped(values);client=self.action
        self.goal_sent=True;self.phase='accepting';self.phase_at=time.monotonic()
        def feedback(m):
            if current():
                self.nav_feedback_at=time.monotonic()
                self.state['distance_remaining']=float(m.feedback.distance_to_goal if direct else m.feedback.distance_remaining)
        future=client.send_goal_async(msg,feedback_callback=feedback)
        def finished(done):
            if not current():return
            try:
                result=done.result().status
                self.goal_handle=None
                if result==GoalStatus.STATUS_ABORTED and self.tf_pause_attempt==attempt:
                    # A delayed abort can arrive after TF recovers. Retry this attempt's
                    # saved absolute target, including for a relative nudge.
                    self.phase='preparing';self.phase_at=time.monotonic()
                    return
                # Retries and recoveries belong to the Nav2 behavior tree. An
                # abort here means they are exhausted: end the task.
                reason='目标已到达，等待底盘静止' if result==4 else nav2_failure_reason(nav_log,result,log_offset)
                self.stop(reason,'succeeded' if result==4 else 'failed',recoverable=True)
            except Exception as exc:self.stop(str(exc),'fault',recoverable=True)
        def accepted(f):
            try:
                handle=f.result()
                if not handle.accepted:
                    if current():
                        if self.tf_pause_attempt==attempt:
                            self.phase='preparing';self.phase_at=time.monotonic()
                        else:self.stop('Nav2 未接受目标','failed',recoverable=True)
                    return
                if not current():
                    handle.cancel_goal_async()
                    return
                self.goal_handle=handle;self.phase='running';self.cmd=(0.,0.,0.)
                self.cmd_at=time.monotonic();self.nav_feedback_at=self.cmd_at
                self.state['status']='真机导航中'
                handle.get_result_async().add_done_callback(finished)
            except Exception as exc:
                if current():self.stop(str(exc),'fault',recoverable=True)
        future.add_done_callback(accepted)

    def handle_command(self, request, response):
        op = request.operation
        try:
            if op == 'stop':
                response.task_id = self.lease.task or ''
                self.stop()
            elif op == 'localize':
                self.localize(dict(owner=request.owner, x=request.x, y=request.y, yaw=request.yaw))
            elif op in ('goal', 'nudge'):
                # Validate zero speed before acquiring ownership or changing task state.
                controller_parameters(request.speed)
                coords = (('dx', 'dy', 'dyaw') if op == 'nudge' else ('x', 'y', 'yaw'))
                data = dict(zip(coords, (request.x, request.y, request.yaw)))
                self.begin(dict(data, owner=request.owner, speed=request.speed), relative=op == 'nudge')
                response.task_id = self.lease.task
            else:
                raise ValueError('未知导航命令：' + op)
            response.accepted = True
        except Exception as exc:
            response.reason = str(exc)
            self.state['error'] = str(exc)
            self.event(str(exc))
        response.localization_id = self.localization_id
        return response

    def _tick(self):
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds*1e-9
        epoch=self.sensors.geometry.continuity_epoch
        if epoch!=self.sensor_epoch:
            self.sensor_epoch=epoch
            self.quality.reset()
            self.initial_tf_ns=None;self.initial_tf_error=None
            if self.lease.active:self.stop('定位数据中断，任务已取消','fault',recoverable=True)
            if self.initial_message:self.finish_localization('failed','定位历史发生中断，请重新初始化')
            self.amcl_at=-1e20;self.amcl_stamp=-1e20
            self.convergence.reset()
            self.localization.update(status='invalidated',reason='定位历史发生中断，需重新初始化')
            self.state.update(initialized=False,pose=None,status='定位数据已恢复，请重新初始化位置与朝向')
            self.event('定位历史发生中断：保留里程计原点，禁止自动恢复任务')
        self.refresh_initialization_transform()
        if now-self.last_lifecycle>1 and not self.lifecycle_pending and self.lifecycle.service_is_ready():
            self.last_lifecycle=now;self.lifecycle_pending=True
            self.poll_amcl_lifecycle()
            def reply(f):
                self.lifecycle_pending=False
                try:self.active_lifecycle=f.result().current_state.id==3;self.lifecycle_at=time.monotonic()
                except Exception:self.active_lifecycle=False
            self.lifecycle.call_async(GetState.Request()).add_done_callback(reply)
        tf_ok=False;tf_age=None
        try:
            t=self.tf.lookup_transform(FRAME['map'],FRAME['base'],rclpy.time.Time())
            stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
            tf_age=ros_now-stamp
            tf_ok=-.35<=tf_age<=.8
            q=t.transform.rotation;p=t.transform.translation
            pose=[p.x,p.y,Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2]]
            self.state['pose']=pose
        except Exception:self.state['pose']=None
        self.state['sensors_ready']=self.sensors.healthy()
        quality_ok=self.quality.tick(now)
        amcl_current=self.quality.amcl_is_current(now)
        confidence_ok=self.confidence.tick(now, quality_ok and tf_ok and amcl_current and self.state['sensors_ready']
                                           and self.quality.monitor.state.get('status')=='trusted'
                                           and self.amcl_active and now-self.amcl_lifecycle_at<3, self.state['initialized'])
        self.state['localization_confidence']=self.confidence.snapshot()
        # AMCL republishes map->odom on every scan, also while standing still.
        # Scan-match quality, covariance and amcl_pose cadence are advisory:
        # they gate initialization and goal submission, never a running task.
        amcl_live=self.amcl_active and now-self.amcl_lifecycle_at<3
        localized=tf_ok and amcl_live
        trusted=bool(localized and quality_ok and amcl_current and confidence_ok)
        self.state['localization_trusted']=trusted
        self.state['localization_warning']=None if trusted or not self.state['initialized'] else self.localization_advisory()
        settled=self.convergence.update(trusted,now)
        healthy=self.state['sensors_ready'] and localized
        executor_alive=(self.mode=='observe' or (now-self.executor_at<1. and not self.executor_state.get('fault') and bool(self.executor_state.get('rights'))))
        execution=executor_alive and (self.mode=='observe' or (now-self.executor_at<.3 and not self.executor_state.get('holding')))
        self.state['ready_checks']=dict(localization_trusted=trusted,amcl_confidence=self.confidence.snapshot(),localization_quality=quality_ok,amcl_update_expected_ok=amcl_current,sensors=self.state['sensors_ready'],tf=tf_ok,tf_age=round(tf_age,3) if tf_age is not None else None,amcl_source_age=round(ros_now-self.amcl_stamp,3),amcl_age=round(now-self.amcl_at,3),amcl_covariance=self.amcl_cov if math.isfinite(self.amcl_cov) else None,executor=bool(execution),lifecycle=self.active_lifecycle,lifecycle_age=round(now-self.lifecycle_at,3),amcl_active=self.amcl_active,amcl_lifecycle_age=round(now-self.amcl_lifecycle_at,3))
        self.state['ready']=bool(self.state['initialized'] and healthy and execution and self.active_lifecycle and now-self.lifecycle_at<3)
        self.state['velocity']=self.sensors.velocity
        self.state['localization_covariance']=self.amcl_cov if math.isfinite(self.amcl_cov) else None
        if self.mode=='mock':self.state['mode_label']='录包回放 · 内存执行器（不控制机器人）'
        elif self.state['replay']:self.state['mode_label']='录包回放 · 只读观察'
        else:self.state['mode_label']='真机观察' if self.mode=='observe' else '真机控制'
        self.state['sensor_health']={k:round(now-v,3) for k,v in self.sensors.received.items()}
        self.state['executor']=self.executor_state
        self.state['applied_command_velocity']=self.executor_state.get('command',[0.,0.,0.]) if 0<=now-self.executor_at<.3 else [0.,0.,0.]
        self.state['owner']=self.lease.owner;self.state['phase']=self.phase
        if self.initial_message:
            if self.sensors.error:
                self.finish_localization('failed','定位初始化失败：'+self.sensors.error)
            elif now-self.initial_at>30:
                reasons=self.localization_reasons()
                if self.amcl_stamp<=self.initial_stamp:reasons.append('未收到本次初始化之后的新鲜 AMCL 位姿')
                if not tf_ok:reasons.append('地图定位变换无效')
                if not confidence_ok:reasons.append(self.confidence.details.get('reason') or '定位置信度等待稳定确认')
                if not amcl_current:reasons.append('AMCL 未按运动门槛输出新结果')
                if not quality_ok:reasons.append(self.state.get('localization_quality',{}).get('reason') or '扫描与地图未匹配')
                self.finish_localization('failed','定位初始化超时：'+'；'.join(reasons or ['定位尚未稳定']))
            elif self.amcl_stamp>self.initial_stamp and healthy and settled:
                self.finish_localization('succeeded','定位就绪；观察模式' if self.mode=='observe' else '定位就绪，可以开始导航')
        elif self.state['initialized'] and self.sensors.error:
            reason='定位或传感器失效，请重新初始化'
            self.state.update(initialized=False,status=reason)
            self.localization.update(status='invalidated',reason=reason)
        elif self.state['initialized'] and not self.state['sensors_ready'] and not self.state['busy']:
            self.state['status']='定位有效；等待新鲜传感器数据，暂不允许运动'
        elif self.state['initialized'] and healthy and not self.state['busy'] and self.phase=='idle':
            if not execution:
                self.state['status']='定位有效；执行器未就绪：'+str(self.executor_state.get('fault') or ('正在确认底盘静止' if self.executor_state.get('holding') else '状态超时/无控制权'))
            elif not trusted:
                self.state['status']='定位质量告警：'+self.state['localization_warning']+'；通过后才能提交新目标'
            else:
                self.state['status']='定位就绪；观察模式' if self.mode=='observe' else '定位就绪，可以开始导航'
        self.state['localization']=dict(self.localization)
        if self.lease.active:
            # Supervision only. Nav2 drives the executor directly; the supervisor
            # cancels the goal when a hard condition stays bad for 1 s.
            hard=dict(sensors=self.state['sensors_ready'] and not self.sensors.error,amcl=amcl_live,
                      executor=executor_alive,nav2=self.active_lifecycle and now-self.lifecycle_at<3)
            # Hold only the chassis. An active Nav2 action keeps its path and controller.
            pause=not tf_ok or (self.tf_paused and not all(hard.values()))
            if pause!=self.tf_paused or pause!=self.executor_state.get('tf_paused',False):
                self.send('tf_pause',paused=pause)
            if pause!=self.tf_paused:
                self.tf_paused=pause
                if pause:self.tf_pause_attempt=self.goal_attempt
                else:self.phase_at=now
                self.event('地图 TF 滞后，暂停并保留目标' if pause else '地图 TF 已恢复，继续原目标')
            if self.route_progress and (pause or self.phase!='running' or not all(hard.values())):
                self.route_progress.pause()
            if all(hard.values()):self.fault_since=None
            elif getattr(self,'fault_since',None) is None:self.fault_since=now
            if self.fault_since is not None and now-self.fault_since>=1.:
                reasons=[]
                if not hard['sensors']:
                    ages={k:round(ros_now-v,3) for k,v in self.sensors.stamps.items()}
                    reasons.append(self.sensors.error or '传感器断流，源数据年龄='+json.dumps(ages,ensure_ascii=False))
                if not hard['amcl']:reasons.append('AMCL 未激活或状态超时')
                if not hard['executor']:reasons.append('执行器故障：'+str(self.executor_state.get('fault') or '状态超时/无控制权'))
                if not hard['nav2']:reasons.append('Nav2 未激活或状态超时')
                self.fault_since=None
                self.stop('；'.join(reasons),'fault',recoverable=not self.executor_state.get('fault'))
            elif self.tf_paused:
                self.state.update(status='等待地图 TF，已保留导航目标',command_velocity=[0.,0.,0.])
            elif self.phase=='preparing':
                if now-self.phase_at>15:self.stop('控制器速度参数设置超时','fault',recoverable=True)
                elif self.params_ok:self.submit_goal()
            elif self.phase=='accepting':
                if now-self.phase_at>15:self.stop('Nav2 未在 15 秒内接受目标','fault',recoverable=True)
            elif self.phase=='running':
                command=self.cmd if now-self.cmd_at<.5 else (0.,0.,0.)
                self.state['command_velocity']=list(command)
                self.state['status']='控制器输出零速度：正在确认朝向、路径或碰撞条件' if not any(command) else '真机导航中'
                if self.route_progress and all(hard.values()) and self.state['pose'] is not None:
                    pose,goal=self.state['pose'],self.state['goal']
                    yaw_error=None
                    if math.dist(pose[:2],goal[:2])<=.50:
                        yaw_error=abs(math.remainder(goal[2]-pose[2],2*math.pi))
                    if self.route_progress.stalled(now,self.odom_pose(),yaw_error):
                        seconds=math.ceil(self.route_progress.elapsed)
                        self.stop(f'连续 {seconds} 秒未取得有效行驶进展，结束本次导航','failed',recoverable=True)

        mean_motion=self.sensors.settling.mean(now)
        self.state['warnings']=[]
        if self.executor_state.get('warning') and self.phase!='paused':self.state['warnings'].append(self.executor_state['warning'])
        if (self.executor_state.get('stationary',False) and mean_motion is not None and
            mean_motion[0]<.03 and mean_motion[1]<.03 and
            (self.sensors.planar_speed>=.02 or abs(self.sensors.velocity[2])>=.03)):
            self.state['warnings'].append('里程计瞬时抖动；SDK 位置稳定，仅告警')
        if time.monotonic()-now>.05:
            self.state['warnings'].append('导航监督处理超过 50 ms；仅性能告警')
        if self.state.get('warning'):self.state['warnings'].append(self.state['warning'])
        if self.phase=='stopping':
            # A 1 s LIO window rejects sustained motion without treating each
            # noisy derivative as movement; SDK position stability is independent.
            motion=self.sensors.settling.mean(now)
            still=(self.sensors.healthy() and motion is not None and motion[0]<.03 and motion[1]<.03
                   and (self.mode=='observe' or (0<=now-self.executor_at<.3 and self.executor_state.get('stationary',False))))
            pending=[]
            if not self.sensors.healthy():pending.append('传感器数据就绪')
            if motion is None:pending.append('建立一秒里程计停稳窗口')
            elif not (motion[0]<.03 and motion[1]<.03):pending.append('里程计均速低于 0.03 m/s、0.03 rad/s')
            if self.mode!='observe':
                if not 0<=now-self.executor_at<.3:pending.append('新鲜底盘反馈')
                elif not self.executor_state.get('stationary',False):pending.append('SDK 确认底盘位置稳定')
            self.state['stop_checks']=dict(lio_mean_speed=motion,sdk_stationary=self.executor_state.get('stationary',False),
                sensors_ready=self.sensors.healthy(),sdk_feedback_age=round(now-self.executor_at,3),
                sdk_holding=self.executor_state.get('holding',False),pending=pending,
                lio_below_limit=bool(motion is not None and motion[0]<.03 and motion[1]<.03))
            if still and self.result_pending=='succeeded' and amcl_live and not tf_ok:
                self.state['status']='已停稳，等待地图 TF 确认到达目标'
            elif still:
                if self.still_since is None:self.still_since=now
                elif now-self.still_since>=.5:
                    if self.result_pending=='succeeded' and (not localized or not self.reached()):
                        self.result_pending='failed'
                        self.state['error']='停车后位姿超出目标容差'
                    self.phase='idle';self.state.update(busy=False,result=self.result_pending,status='已静止 · '+str(self.result_pending))
                    if self.state['error']=='未能确认停车，请检查现场状态':self.state['error']=None
            else:self.still_since=None
            if self.phase=='stopping' and not still and now-self.phase_at>5:self.state['error']='未能确认停车，请检查现场状态'
        if self.sensors.error:self.state['error']=self.sensors.error
        if now-self.last_sample>.2:
            self.last_sample=now
            self.sensors.metrics.add('supervisor_tick_ms',(time.monotonic()-now)*1000)
            if 'pipeline' not in self.state or now-self.last_metrics>=1:
                self.last_metrics=now
                self.state['pipeline']=self.sensors.metrics.snapshot()
            self.state['pipeline']['queue']=self.sensors.worker.state()
            self.state['pipeline']['source_age_s']={k:round(ros_now-v,3) for k,v in self.sensors.stamps.items()}

            self.state.update(phase='paused' if self.tf_paused else self.phase, sensor_error=self.sensors.error,
                              scan_reference_ns=self.sensors.latest_scan_stamp_ns,
                              scan_source_stamp=self.sensors.latest_scan_source_stamp)
            if self.state_pub.get_subscription_count():
                self.state_pub.publish(String(data=json.dumps(self.state, ensure_ascii=False)))

            self.log.write(json.dumps(dict(time=time.time(),task=self.state['task'],phase=self.state['phase'],pose=self.state['pose'],velocity=self.sensors.velocity,command=self.state['command_velocity'],result=self.state['result'],route_execution=self.state.get('route_execution'),executor=dict(self.executor_state),stop_checks=self.state.get('stop_checks'),recovery=self.state.get('recovery'),localization_quality=self.state.get('localization_quality'),localization_confidence=self.state.get('localization_confidence')),ensure_ascii=False)+'\n')
