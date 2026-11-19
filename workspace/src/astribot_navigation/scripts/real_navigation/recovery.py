"""Bounded retreat state machine and conservative swept clearance, no SDK/ROS."""
import math
import numpy as np
from .geometry import safe_pose


def recoverable_navigation_failure(status,reason,attempts):
    return status==6 and attempts<2 and any(text in reason for text in
        ('预测到前方碰撞','避障余量','可行轨迹','未取得进展','转弯半径'))


def backup_clearance(grid,pose,footprint,padding,distance,clouds,twist=(0.,0.,0.)):
    # Extra space covers remaining SDK target lead plus one control interval.
    sweep=distance+.15
    vx,vy,w=twist
    poly=np.asarray(footprint,float)
    low=poly.min(axis=0)-padding;high=poly.max(axis=0)+padding
    # Pure recovery retreats retain the original rear-only sweep. For a
    # reversing arc include the actual lateral and rotational displacement.
    curved=vx<0 and (vy!=0 or w!=0)
    if curved:
        duration=distance/-vx
        steps=max(1,math.ceil(max(math.hypot(vx,vy)*duration,abs(w)*duration)/.01))
        offsets=[]
        x=y=yaw=0.
        dt=duration/steps
        for i in range(steps+1):
            offsets.append((x,y,yaw))
            mid=yaw+.5*w*dt
            x+=dt*(vx*math.cos(mid)-vy*math.sin(mid))
            y+=dt*(vx*math.sin(mid)+vy*math.cos(mid))
            yaw+=w*dt
    else:
        offsets=[(-d,0.,0.) for d in np.linspace(0.,sweep,max(2,int(math.ceil(sweep/.01))+1))]
    c,s=math.cos(pose[2]),math.sin(pose[2])
    for x,y,yaw in offsets:
        p=[pose[0]+c*x-s*y,pose[1]+s*x+c*y,pose[2]+yaw]
        if not safe_pose(grid,p,footprint,padding):return False,'后退扫掠区域在地图上有障碍或未知格'
    if not curved:
        low[0]-=sweep
        high[0]=poly[:,0].max()
    else:
        low[0]-=.15
        # Every swept corner stays within the corner radius plus origin travel.
        radius=np.linalg.norm(np.maximum(np.abs(low),np.abs(high)))+math.hypot(vx,vy)*duration
        radius_squared=(radius+1e-9)**2
    for points in clouds:
        points=np.asarray(points,float)
        if points.ndim!=2 or points.shape[1]!=3 or len(points)<20 or not np.isfinite(points).all():
            return False,'缺少有效的完整实时障碍点云'
        if curved:
            xy=points[:,:2]
            points=points[np.einsum('ij,ij->i',xy,xy)<=radius_squared]
            if not len(points):continue
        for x,y,yaw in (offsets if curved else [(0.,0.,0.)]):
            c,s=math.cos(yaw),math.sin(yaw)
            local=(points[:,:2]-[x,y])@np.array([[c,-s],[s,c]])
            if np.any(np.all((local>=low)&(local<=high),axis=1)):
                return False,'实时雷达检测到后退扫掠区域有障碍'
    if len(clouds)!=2:return False,'需要前后雷达共同确认回退空间'
    return True,None


class Retreat:
    def __init__(self,now,distance=.15,speed=.03):
        self.started=now;self.distance=distance;self.speed=speed
        self.phase='stopping';self.since=None;self.origin=None;self.progress=0.

    def step(self,now,feedback,stationary):
        if now-self.started>=12:raise TimeoutError('本次回退恢复超过 12 秒，等待重新规划')
        if len(feedback)!=3 or not all(math.isfinite(x) for x in feedback):
            raise RuntimeError('回退恢复缺少有效底盘反馈')
        if self.phase in ('stopping','settling'):
            if not stationary:self.since=None
            elif self.since is None:self.since=now
            elif now-self.since>=.3:
                if self.phase=='stopping':
                    self.origin=list(feedback);self.phase='backing';self.since=None
                else:self.phase='replanning'
            return 0.
        if self.phase=='backing':
            self.progress=max(0.,self.origin[0]-feedback[0])
            if (feedback[0]-self.origin[0]>.02 or abs(feedback[1]-self.origin[1])>.03
                    or abs(feedback[2]-self.origin[2])>.05):
                raise RuntimeError('回退过程中底盘方向或侧向偏差异常')
            if self.progress>=self.distance:
                self.phase='settling';self.since=None;return 0.
            return -self.speed
        return 0.

    def snapshot(self):
        return dict(phase=self.phase,distance=self.distance,progress=round(self.progress,3),speed=self.speed,timeout=12)
