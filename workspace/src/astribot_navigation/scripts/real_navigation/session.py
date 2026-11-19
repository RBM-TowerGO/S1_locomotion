"""Owned process supervisor for live observation, validated driving, and bag replay."""
import argparse
import fcntl
import datetime
import json
import os
from pathlib import Path
import signal
import sys
import time
import uuid
import yaml
from .core import (validate_config,digest,costmap_inflation_radius,GOAL_XY,GOAL_YAW,
                   MAX_ANGULAR_SPEED)

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
WORKSPACE=REPO.parents[1]


def nav_params(config,mapfile,replay):
    base=yaml.safe_load((REPO/'astribot_nav_bringup/config/nav2.yaml').read_text())
    web=yaml.safe_load((REPO/'astribot_nav_bringup/config/web_controller.yaml').read_text())
    for name in ('planner_server','controller_server'):
        base[name]['ros__parameters'].update(web[name]['ros__parameters'])
    def rewrite(obj):
        if isinstance(obj,dict):
            for key,value in obj.items():
                if key=='use_sim_time':obj[key]=replay
                elif isinstance(value,str):
                    obj[key]={'base_link':'tg_nav_base','odom':'tg_nav_odom','map':'tg_nav_map','/scan':'/towergo/nav/scan','/odom':'/towergo/nav/odom'}.get(value,value)
                else:rewrite(value)
        elif isinstance(obj,list):
            for value in obj:rewrite(value)
    rewrite(base)
    base['map_server']['ros__parameters'].update(yaml_filename=str(mapfile),frame_id='tg_nav_map')
    base['amcl']['ros__parameters'].update(update_min_d=config.get('amcl_update_min_d',0.),update_min_a=config.get('amcl_update_min_a',0.),transform_tolerance=.1,robot_model_type='towergo::LioPlanarMotionModel')
    inflation=costmap_inflation_radius(config)
    ps=base['planner_server']['ros__parameters']
    # Stock 2D planner: one cost-aware grid search, no motion primitives,
    # turning radius, straight-line shortcut or post-hoc footprint rejection.
    # The robot turns in place, so the path only needs to keep the centre out
    # of inscribed cost; the controller checks the full rectangle.
    ps['GridBased']=dict(plugin='nav2_smac_planner/SmacPlanner2D',tolerance=.10,
                         downsample_costmap=False,downsampling_factor=1,allow_unknown=False,
                         max_iterations=1000000,max_on_approach_iterations=1000,max_planning_time=2.0,
                         cost_travel_multiplier=2.0,use_final_approach_orientation=False,
                         smoother=dict(max_iterations=1000,w_smooth=.3,w_data=.2,tolerance=1e-10))
    cp=base['controller_server']['ros__parameters']
    horizon=config.get('collision_time_horizon',1.5)
    cp['controller_plugins']=['FollowPath','Nudge']
    # One controller owns path tracking: RPP turns in place when the path is
    # behind, otherwise drives forward with curvature/obstacle/approach
    # slow-down and a full-footprint collision projection. No lateral or
    # reverse sampling, so the executor never sees sign-flipping vy/vx.
    cp['FollowPath']=dict(
        plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        desired_linear_vel=.05,  # Replaced per goal by controller_parameters().
        lookahead_dist=.5,min_lookahead_dist=.3,max_lookahead_dist=.9,lookahead_time=1.5,
        use_velocity_scaled_lookahead_dist=True,use_interpolation=True,
        use_rotate_to_heading=True,rotate_to_heading_min_angle=.6,
        rotate_to_heading_angular_vel=MAX_ANGULAR_SPEED,max_angular_accel=.6,
        allow_reversing=False,goal_dist_tol=GOAL_XY,
        use_collision_detection=True,max_allowed_time_to_collision_up_to_carrot=horizon,
        use_regulated_linear_velocity_scaling=True,regulated_linear_scaling_min_radius=.9,
        regulated_linear_scaling_min_speed=.03,
        use_cost_regulated_linear_velocity_scaling=True,cost_scaling_dist=.6,cost_scaling_gain=1.,
        inflation_cost_scaling_factor=5.,
        min_approach_linear_velocity=.03,approach_velocity_scaling_dist=.6,
        transform_tolerance=.3)
    cp['Nudge']=dict(plugin='towergo::OmniController',desired_linear_vel=.05,collision_time_horizon=horizon)
    # A 180 deg in-place turn at 0.3 rad/s takes ~11 s without XY progress.
    cp['progress_checker'].update(required_movement_radius=.05,movement_time_allowance=20.)
    cp['odom_topic']='/towergo/nav/odom'
    cp['goal_checker'].update(plugin='nav2_controller::SimpleGoalChecker', stateful=False,
                              xy_goal_tolerance=GOAL_XY,yaw_goal_tolerance=GOAL_YAW)
    for name in ('global_costmap','local_costmap'):
        p=base[name][name]['ros__parameters']
        p['footprint']=json.dumps(config['footprint']);p['footprint_padding']=config['footprint_padding']
        p['track_unknown_space']=True
        p['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_topic':'/towergo/nav/map','map_subscribe_transient_local':True}
        p['inflation_layer'].update(inflation_radius=inflation,cost_scaling_factor=5.)
        if name=='global_costmap':
            # Static map only. The obstacle clouds exclude floor returns, so in
            # open space no ray ever clears a marked cell; in this non-rolling
            # map stray returns persisted, invalidated the path and forced
            # detours. Live obstacles stay in the rolling local costmap.
            p['plugins']=['static_layer','inflation_layer']
            p.pop('obstacle_layer',None)
            continue
        p['plugins']=['static_layer','obstacle_layer','inflation_layer']
        layer=p['obstacle_layer']=dict(plugin='nav2_costmap_2d::ObstacleLayer',observation_sources='front back')
        # The bridge classifies physical returns relative to the nearby floor.
        # Nav2's second height test runs in the global costmap frame. On a
        # slope it could reject a valid low return, so leave that gate open.
        relative_ground='ground_z_in_base' in config
        if relative_ground:
            layer['min_obstacle_height']=-1.e6
            layer['max_obstacle_height']=1.e6
        for sensor in ('front','back'):
            layer[sensor]=dict(topic='/towergo/nav/obstacles/'+sensor,data_type='PointCloud2',
                               marking=True,clearing=True,obstacle_max_range=2.5,raytrace_max_range=3.0,
                               min_obstacle_height=-1.e6 if relative_ground else config['obstacle_min_height'],
                               max_obstacle_height=1.e6 if relative_ground else config['obstacle_max_height'])
    bt=base['bt_navigator']['ros__parameters']
    # Action acknowledgement is not a sensor watchdog. On the loaded Orin,
    # the generic 20 ms budget can abort a healthy concurrent replan.
    bt.update(default_server_timeout=1000,bt_loop_duration=50)
    bt['default_nav_through_poses_bt_xml']=str(REPO/'astribot_nav_bringup/config/navigation_through.xml')
    bt['default_nav_to_pose_bt_xml']=str(HERE/'navigation.xml')
    bt['plugin_lib_names']=['nav2_compute_path_through_poses_action_bt_node','nav2_compute_path_to_pose_action_bt_node','nav2_follow_path_action_bt_node','nav2_pipeline_sequence_bt_node',
        'nav2_rate_controller_bt_node','nav2_is_path_valid_condition_bt_node',
        'nav2_globally_updated_goal_condition_bt_node','nav2_path_expiring_timer_condition',
        # Stock bounded recovery: clear local costmap / wait, then abort.
        # ReactiveFallback is a BehaviorTree.CPP builtin: no Nav2 library to load.
        'nav2_recovery_node_bt_node','nav2_round_robin_node_bt_node',
        'nav2_goal_updated_condition_bt_node','nav2_clear_costmap_service_bt_node','nav2_wait_action_bt_node']
    return {'towergo':{'nav':base}}


def configure_lio_output(lio_parameters,config):
    """Select FAST-LIO cloud output for the configured AMCL scan source."""
    import numpy as np
    p=lio_parameters['/**']['ros__parameters']
    if config.get('scan_input','lio_body')=='lio_body':
        # The body cloud is only published together with the (downsampled)
        # world cloud. Dense output stays off; the body cloud is always the
        # full deskewed scan regardless of dense_publish_en.
        p['publish'].update(scan_publish_en=True,scan_bodyframe_pub_en=True,dense_publish_en=False)
        # Body points are converted back with navigation.yaml imu_from_lidar;
        # both extrinsic copies must describe the same transform.
        imu=np.asarray(config['imu_from_lidar'],float)
        rotation=np.asarray(p['mapping']['extrinsic_R'],float).reshape(3,3)
        translation=np.asarray(p['mapping']['extrinsic_T'],float)
        if not (np.allclose(rotation,imu[:3,:3],atol=1e-6) and np.allclose(translation,imu[:3,3],atol=1e-6)):
            raise ValueError('FAST-LIO 外参与 navigation.yaml imu_from_lidar 不一致')
    else:
        # Raw-pair mode derives scans from raw sensors; skip cloud serialization.
        p['publish']['scan_publish_en']=False


def start_live_sensors(node, replay, pump):
    from mapping_support import REQUIRED_RAW_TOPICS
    from tg_mapping import lidar_control
    if replay:
        return 'replay'
    # Same startup policy as keyboard SLAM: reuse existing drivers;
    # never toggle them merely because one input is temporarily missing.
    if any(node.count_publishers(topic)>0 for topic in REQUIRED_RAW_TOPICS):
        return 'existing'
    print('正在开启双雷达及前 IMU…', flush=True)
    message=lidar_control(node, True, pump=pump)
    print('传感器服务：'+message+'；等待实际数据', flush=True)
    return 'requested'


def parser():
    p=argparse.ArgumentParser(description='S1 真机导航，默认只观察，不加载 SDK')
    p.add_argument('--map',default=str(WORKSPACE/'data/maps/manual_edit_20261105_speckle_fill/map.yaml'))
    p.add_argument('--config',default=str(HERE/'navigation.yaml'))
    p.add_argument('--mode',choices=['observe','hardware','mock'],default='observe')
    p.add_argument('--replay',help='原始双雷达/IMU bag；隔离 Domain 125，不加载 SDK')
    p.add_argument('--replay-rate',type=float,default=1.,help='隔离回放倍率（0.5–1）')
    p.add_argument('--duration',type=float,default=0,help='验证会话最长秒数；0 为持续运行')
    p.add_argument('--output-root',default=str(WORKSPACE/'data/reports/real_navigation'))
    p.add_argument('--diagnostic-seconds',type=float,default=600.,help='完整诊断录包时长；0 禁用，默认 600 秒')
    p.add_argument('--diagnostic-max-bytes',type=int,default=2147483648,help='诊断录包总大小上限（默认 2 GiB）')
    p.add_argument('--check',action='store_true',help='检查配置和依赖，不启动节点')
    return p


def main():
    args=parser().parse_args()
    if not .5<=args.replay_rate<=1:raise ValueError('回放倍率必须在 0.5–1；更慢会超出逐帧等待期限')
    if args.diagnostic_seconds<0 or args.diagnostic_max_bytes<0:raise ValueError('诊断记录限制必须非负')
    config=yaml.safe_load(Path(args.config).read_text())
    validate_config(config,args.mode=='hardware')
    inflation=float(config.get('inflation_radius',.2))
    if not 0<=inflation<=2:raise ValueError('膨胀半径必须在 0–2 m')
    if args.mode=='mock':
        if not args.replay:raise ValueError('mock 仅用于隔离录包回放')
        config['max_speed']=.3
    if args.replay and args.mode=='hardware':raise ValueError('回放禁止硬件后端')
    mapfile=Path(args.map).resolve()
    m=yaml.safe_load(mapfile.read_text());image=mapfile.parent/m['image']
    if abs(m['resolution']-.05)>1e-8:raise ValueError('当前 lattice 配置要求 0.05 m 地图')
    from ament_index_python.packages import get_package_prefix,get_package_share_directory
    dependencies=['fast_lio','towergo_dual_livox','astribot_nav_bringup','astribot_nav_interfaces','nav2_amcl','nav2_controller']
    for package in dependencies:get_package_prefix(package)
    if args.check:
        print(json.dumps(dict(mode=args.mode,map=str(mapfile),max_speed=config['max_speed'],dependencies=dependencies),ensure_ascii=False,indent=2));return
    if os.environ.get('ROS_DOMAIN_ID')!=('125' if args.replay else '25'):raise ValueError('错误的 ROS Domain，请使用 tg-navigation.sh')
    # A new session replaces any previous one (normal stop first, then forced).
    from .takeover import replace_previous,claim
    session_lock=replace_previous(os.environ['ROS_DOMAIN_ID'],log=lambda text:print(text,flush=True))
    from mapping_support import OwnedProcess,atomic_json,REQUIRED_RAW_TOPICS
    from tg_mapping import parameters,topic_names
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import SingleThreadedExecutor
    from .supervisor import NavigationSupervisor
    run=Path(args.output_root)/('navigation_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    run.mkdir(parents=True)
    claim(session_lock,run)
    (run/'config.yaml').write_text(yaml.safe_dump(config));(run/'nav2.yaml').write_text(yaml.safe_dump(nav_params(config,mapfile,bool(args.replay))))
    report=dict(mode=args.mode,replay=args.replay,map=str(mapfile),map_sha256=digest(mapfile),image_sha256=digest(image),
                config_sha256=digest(run/'config.yaml'),max_speed=config['max_speed'],started=time.time(),status='starting',processes={})
    procs={};node=None;executor=None;interrupted=False
    def spawn(name,cmd):
        proc=OwnedProcess(name,cmd,log=run/(name+'.log'))
        procs[name]=proc;report['processes'][name]={'pid':proc.proc.pid,'command':cmd}
        return proc
    def halt(sig,frame):
        nonlocal interrupted
        interrupted=True
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,halt)
    rclpy.init(args=['--ros-args','-r','/tf:=/towergo/nav/tf','-r','/tf_static:=/towergo/nav/tf_static',
                      '-p','use_sim_time:='+str(bool(args.replay)).lower()],signal_handler_options=SignalHandlerOptions.NO)
    try:
        node=NavigationSupervisor(config,str(mapfile),args.mode,run)
        # Keep the node attached: rclpy.spin_once(node) removes/re-adds it
        # on every callback and repeatedly rebuilds this large wait set.
        executor=SingleThreadedExecutor();executor.add_node(node)
        # Discover existing publishers before reusing the keyboard SLAM sensor startup.
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
        # A replaced session's DDS participants can linger briefly after exit.
        def previous_publishers():
            return node.count_publishers('/towergo/nav/odom')>1 or node.count_publishers('/towergo/nav/lio/odometry')
        deadline=time.monotonic()+20
        while previous_publishers() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.1)
        if previous_publishers():
            raise RuntimeError('旧导航会话的话题仍在发布；请检查是否有其他用户或终端启动的导航进程')
        if args.replay and (node.count_publishers('/clock') or any(node.count_publishers(t) for t in REQUIRED_RAW_TOPICS)):
            raise RuntimeError('回放 Domain 已有传感器或时钟发布者')
        report['sensor_startup']=start_live_sensors(
            node, bool(args.replay), lambda: executor.spin_once(timeout_sec=.05))
        # Device service success means requested, not that sensor data is ready.
        # The existing live-data health checks determine navigation readiness.
        topics=topic_names('/towergo/nav/lio')
        base=Path(get_package_share_directory('fast_lio'))/'config/mid360.yaml'
        lio_parameters=parameters(base,topics,bool(args.replay))
        # Navigation derives both obstacle clouds directly from raw sensors.
        # The registered visualization cloud is unused; avoid transforming and
        # serializing it on FAST-LIO's real-time estimation thread.
        configure_lio_output(lio_parameters,config)
        (run/'lio.yaml').write_text(yaml.safe_dump(lio_parameters))
        sim=str(bool(args.replay)).lower()
        adapter=Path(get_package_prefix('towergo_dual_livox'))/'lib/towergo_dual_livox/dual_adapter'
        cmd=[str(adapter),'--ros-args','-p','imu_acceleration_unit:=g','-p','use_sim_time:='+sim,
             '-p','drop_invalid_timestamp_frames:=true',
             '-p','coordinate_contract:=both_raw_topics_already_in_front_lidar_frame','-p','reliable_input:='+sim,
             '-r','__node:=navigation_dual_adapter']
        for old,new in {'/towergo/lio/lidar':topics['lidar'],'/towergo/lio/imu':topics['imu'],'/towergo/mapping_fault':topics['fault'],'/diagnostics':topics['diagnostics']}.items():cmd+=['-r',old+':='+new]
        spawn('adapter',cmd)
        fast=Path(get_package_prefix('fast_lio'))/'lib/fast_lio/fastlio_mapping'
        cmd=[str(fast),'--ros-args','--params-file',str(run/'lio.yaml'),'-r','__node:=navigation_fastlio']
        for old,key in [('/Odometry','odom'),('/cloud_registered','cloud'),('/cloud_registered_body','body'),('/tf','tf'),('/tf_static','tf_static')]:cmd+=['-r',old+':='+topics[key]]
        spawn('fastlio',cmd)
        spawn('nav2',['ros2','launch',str(HERE/'launch.py'),'params:='+str(run/'nav2.yaml')])
        if args.mode in ('hardware','mock'):spawn('executor',[sys.executable,'-m','real_navigation.executor',str(run/'config.yaml'),args.mode])
        if args.diagnostic_seconds and args.diagnostic_max_bytes:
            diagnostic_topics=[*REQUIRED_RAW_TOPICS,'/towergo/nav/lio/odometry','/towergo/nav/odom',
                '/towergo/nav/scan','/towergo/nav/amcl_pose','/towergo/nav/tf','/towergo/nav/tf_static',
                '/towergo/nav/executor_status','/towergo/nav/executor_control','/towergo/nav/cmd_vel_nav','/towergo/nav/route_execution',
                '/astribot_chassis/joint_space_states','/towergo/nav/map','/clock',
                # Local obstacle evidence: what DWB actually avoided.
                '/towergo/nav/local_costmap/costmap','/towergo/nav/local_costmap/published_footprint',
                '/towergo/nav/plan','/towergo/nav/obstacles/front','/towergo/nav/obstacles/back']
            qos={t:dict(reliability='best_effort',durability='volatile',history='keep_last',depth=10)
                 for t in diagnostic_topics}
            for t in ('/towergo/nav/tf_static','/towergo/nav/map'):
                qos[t]=dict(reliability='reliable',durability='transient_local',history='keep_last',depth=1)
            (run/'diagnostic_qos.yaml').write_text(yaml.safe_dump(qos))
            spawn('diagnostic_recorder',['ros2','bag','record','-s','mcap','-o',str(run/'diagnostic_bag'),
                '--qos-profile-overrides-path',str(run/'diagnostic_qos.yaml'),
                '--max-bag-size','268435456','--max-cache-size','67108864',*diagnostic_topics])
            report['diagnostics']=dict(status='recording',seconds=args.diagnostic_seconds,max_bytes=args.diagnostic_max_bytes)
        if args.replay:
            spawn('player',['ros2','bag','play',str(Path(args.replay).resolve()),'--clock','--rate',str(args.replay_rate),'--delay','2','--topics',*REQUIRED_RAW_TOPICS])
        print('真机导航 '+args.mode+' · ROS 服务 /towergo/nav/command\n记录目录：'+str(run),flush=True)
        report['status']='running';atomic_json(run/'session.json',report)
        diagnostic_check=0.;diagnostic_stopping=False
        start=time.monotonic();maxima={'front':0,'back':0,'scan':0};ready_seen=False
        while rclpy.ok() and not interrupted and (not args.duration or time.monotonic()-start<args.duration):
            executor.spin_once(timeout_sec=.02)
            if 'diagnostic_recorder' in procs and not diagnostic_stopping and time.monotonic()-diagnostic_check>=1:
                diagnostic_check=time.monotonic()
                size=sum(p.stat().st_size for p in (run/'diagnostic_bag').glob('*') if p.is_file())
                if time.monotonic()-start>=args.diagnostic_seconds or size>=args.diagnostic_max_bytes:
                    proc=procs['diagnostic_recorder'].proc
                    if proc.poll() is None:proc.send_signal(signal.SIGINT)
                    diagnostic_stopping=True;report['diagnostics'].update(status='bounded_stop',bytes=size)
            for name,p in procs.items():
                if p.poll() is not None:
                    if name=='diagnostic_recorder':
                        report['diagnostics']['exit_code']=p.poll()
                        if not diagnostic_stopping:report['diagnostics']['status']='recorder_exited'
                        continue
                    if name=='player' and p.poll()==0:interrupted=True;break
                    log_path = run / (name + '.log')
                    try:
                        tail = '\n'.join(log_path.read_text(errors='replace').splitlines()[-15:])
                    except OSError:
                        tail = '(无法读取日志)'
                    raise RuntimeError(name + ' 进程退出: ' + str(p.poll()) +
                                       '\n日志: ' + str(log_path) + '\n' + tail)
            ready_seen=ready_seen or node.sensors.healthy()
            for key in maxima:
                if key in node.sensors.received:maxima[key]+=1
        report.update(status='complete',sensor_ready_seen=ready_seen,sensor_observation_counts=maxima,
                      localization_initialized=node.state['initialized'],navigation_ready=node.state['ready'],
                      last_state=node.state,validation='observation/replay only; no hardware acceptance')
    except Exception as exc:
        report.update(status='failed',error=str(exc));raise
    finally:
        if node:
            node.destroy_service(node.command_service)
            node.stop('会话结束')
            end=time.monotonic()+.3
            while rclpy.ok() and time.monotonic()<end:executor.spin_once(timeout_sec=.02)
        # Bounded shutdown: the executor gets longer to stop the chassis; the
        # rest escalate to SIGTERM/SIGKILL after 5 s instead of 20 s each.
        for name in ('diagnostic_recorder','executor','player','nav2','fastlio','adapter'):
            if name in procs:
                try:procs[name].stop(timeout=8 if name=='executor' else 5)
                except Exception as exc:report.setdefault('shutdown_errors',[]).append(str(exc))
        if executor:executor.shutdown()
        if node:node.quality.close();node.sensors.close();node.log.close();node.destroy_node()
        rclpy.try_shutdown();report['ended']=time.time();atomic_json(run/'session.json',report)

if __name__=='__main__':main()
