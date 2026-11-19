"""Replace a previous navigation session of the same user and ROS domain.

A new session always wins. The previous owner first gets SIGTERM, which runs
its normal cleanup (the executor stops the chassis and releases control). If
it has not exited within the grace period, every process a navigation session
owns is stopped directly: executor first, then the remaining process groups.
"""
import fcntl
import json
import os
from pathlib import Path
import signal
import time

# Command-line fragments carried only by processes a navigation session owns.
# Livox drivers and other shared sensor processes never match these.
OWNED_MARKERS = (
    'real_navigation.session', 'real_navigation.executor', 'real_navigation/launch.py',
    '__ns:=/towergo/nav', '__node:=navigation_fastlio', '__node:=navigation_dual_adapter',
    '/diagnostic_bag')


def lock_path(domain):
    return '/tmp/towergo-navigation-%s-%s.lock' % (os.getuid(), domain)


def _cmdline(pid):
    try:
        return Path('/proc', str(pid), 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
    except OSError:
        return ''


def _domain(pid):
    try:
        for item in Path('/proc', str(pid), 'environ').read_bytes().split(b'\0'):
            if item.startswith(b'ROS_DOMAIN_ID='):
                return item.split(b'=', 1)[1].decode()
    except OSError:
        pass
    return None


def leftover_processes(domain):
    """Navigation-owned processes of this user in this ROS domain, except us."""
    me, uid, found = os.getpid(), os.getuid(), []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == me:
            continue
        try:
            if entry.stat().st_uid != uid:
                continue
        except OSError:
            continue
        pid = int(entry.name)
        command = _cmdline(pid)
        if any(marker in command for marker in OWNED_MARKERS) and _domain(pid) == domain:
            found.append((pid, command))
    return found


def _alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _send(pid, sig, group=False):
    try:
        if group:
            pgid = os.getpgid(pid)
            if pgid != os.getpgid(0):  # Never signal our own process group.
                os.killpg(pgid, sig)
                return
        os.kill(pid, sig)
    except (ProcessLookupError, PermissionError):
        pass


def _wait(pids, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and any(_alive(p) for p in pids):
        time.sleep(.2)
    return [p for p in pids if _alive(p)]


def _try_lock(handle):
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return True
    except BlockingIOError:
        return False


def _wait_lock(handle, seconds):
    deadline = time.monotonic() + seconds
    while not _try_lock(handle):
        if time.monotonic() >= deadline:
            return False
        time.sleep(.2)
    return True


def stop_leftovers(domain, log=print):
    """Stop navigation-owned processes; executor gets the first, gentlest stop."""
    leftovers = leftover_processes(domain)
    if not leftovers:
        return
    log('清理旧导航会话残留进程：' + ', '.join(str(p) for p, _ in leftovers))
    executors = [p for p, c in leftovers if 'real_navigation.executor' in c]
    # A session main process may share its launching shell's process group.
    sessions = [p for p, c in leftovers if 'real_navigation.session' in c]
    others = [p for p, c in leftovers if p not in executors and p not in sessions]
    for pid in executors:
        _send(pid, signal.SIGINT)  # Executor cleanup stops the chassis.
    for pid in _wait(executors, 8.):
        _send(pid, signal.SIGKILL)
    for pid in sessions:
        _send(pid, signal.SIGKILL)  # Its graceful chance was the SIGTERM above.
    # Owned children run in their own process groups (ros2 launch trees too).
    for pid in others:
        _send(pid, signal.SIGINT, group=True)
    for pid in _wait(others, 8.):
        _send(pid, signal.SIGKILL, group=True)
    remaining = _wait(executors + sessions + others, 5.)
    if remaining:
        raise RuntimeError('旧导航会话进程无法结束：' + ', '.join(map(str, remaining)))


def replace_previous(domain, grace=40., log=print):
    """Return a locked handle, after ending any previous session."""
    path = lock_path(domain)
    handle = open(path, 'a')
    if not _try_lock(handle):
        try:
            owner = json.loads(Path(path).read_text() or '{}').get('pid')
        except (OSError, ValueError):
            owner = None
        # Sessions started before this file existed did not record their PID;
        # a recorded PID is only trusted if it is still a navigation session.
        if not (isinstance(owner, int) and 'real_navigation.session' in _cmdline(owner)):
            owner = None
        owners = [owner] if owner else [p for p, c in leftover_processes(domain)
                                       if 'real_navigation.session' in c]
        log('发现正在运行的导航会话（PID %s），请求其正常结束…' % ', '.join(map(str, owners)))
        for pid in owners:
            _send(pid, signal.SIGTERM)
        if not _wait_lock(handle, grace):
            log('旧会话 %.0f 秒内未结束，强制清理' % grace)
            stop_leftovers(domain, log)
            for pid in owners:
                _send(pid, signal.SIGKILL)
            if not _wait_lock(handle, 5.):
                handle.close()
                raise RuntimeError('无法接管导航会话锁：' + path)
    # Orphans survive when a previous session itself was killed.
    stop_leftovers(domain, log)
    claim(handle)
    return handle


def claim(handle, run=None):
    handle.seek(0)
    handle.truncate()
    handle.write(json.dumps(dict(pid=os.getpid(), run=str(run) if run else None, started=time.time())))
    handle.flush()
