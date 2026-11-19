"""Optional loopback HTTP adapter for the navigation ROS API."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading


def server(client, port):
    html = Path(__file__).with_name('index.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def output(self, code, data, kind='application/json'):
            if isinstance(data, dict):
                data = json.dumps(data, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            route = self.path.partition('?')[0]
            if route == '/':
                return self.output(200, html.replace(b'__TOKEN__', client.token.encode()), 'text/html; charset=utf-8')
            if route == '/map.png':
                with client.lock:
                    png = client.png
                if png is None:
                    return self.output(503, dict(error='等待 ROS 地图'))
                return self.output(200, png, 'image/png')
            if route == '/api/state':
                try:
                    return self.output(200, client.state())
                except ConnectionError as exc:
                    return self.output(503, dict(error=str(exc)))
            self.output(404, {})

        def do_POST(self):
            if self.headers.get('X-Session-Token') != client.token:
                return self.output(403, dict(error='导航服务已重启或页面会话已失效，请刷新整个页面后重试'))
            try:
                size = int(self.headers.get('Content-Length', 0))
                if not 0 < size <= 2048:
                    raise ValueError('Invalid request size')
                self.connection.settimeout(1)
                obj = json.loads(self.rfile.read(size))
                if self.path == '/api/heartbeat':
                    state = client.state()
                    current = state['busy'] and state['task'] == obj['task'] and state['owner'] == obj['owner']
                    return self.output(202, dict(queued=True) if current else dict(superseded=True))
                if self.path not in ('/api/localize', '/api/goal', '/api/nudge', '/api/stop'):
                    return self.output(404, {})
                self.output(202, client.call(self.path[5:], obj))
            except (ValueError, KeyError, TypeError, AssertionError, OverflowError) as exc:
                self.output(400, dict(error=str(exc)))
            except TimeoutError as exc:
                self.output(504, dict(error=str(exc)))
            except (ConnectionError, RuntimeError) as exc:
                self.output(503, dict(error=str(exc)))

    http = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    http.daemon_threads = True
    threading.Thread(target=http.serve_forever, daemon=True).start()
    return http


def main():
    parser = argparse.ArgumentParser(description='连接已有 S1 导航会话的网页客户端')
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--replay', action='store_true', help='连接 Domain 125 的回放会话')
    args = parser.parse_args()
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from .web_client import NavigationClient
    rclpy.init(args=['--ros-args', '-r', '/tf:=/towergo/nav/tf', '-r', '/tf_static:=/towergo/nav/tf_static',
                     '-p', 'use_sim_time:='+str(args.replay).lower()])
    client = NavigationClient()
    http = None
    try:
        http = server(client, args.port)
        print(f'导航网页 http://127.0.0.1:{http.server_port}', flush=True)
        rclpy.spin(client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if http is not None:
            http.shutdown()
            http.server_close()
        client.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
