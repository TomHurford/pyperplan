"""Local planner explorer. Run: .venv/bin/python -m explorer.server"""
import argparse
import json
import multiprocessing as mp
from pathlib import Path
from queue import Empty
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[1]
SEARCHES = ['bfs', 'astar', 'gbf', 'wastar', 'ehs', 'ehc', 'eehc', 'gehc', 'hb_ehc', 'hybrid_ehc', 'rhc']
HEURISTICS = ['hff', 'hcff', 'hadd', 'hmax', 'blind', 'lmcut']


def catalog():
    return {p.name: [t.name for t in sorted(p.glob('task*.pddl'))]
            for p in sorted((ROOT / 'benchmarks').iterdir()) if p.is_dir()}


def validate_config(config):
    domains = catalog()
    if config.get('domain') not in domains or config.get('problem') not in domains[config['domain']]:
        raise ValueError('Choose a listed benchmark problem')
    if config.get('search') not in SEARCHES or config.get('heuristic') not in HEURISTICS:
        raise ValueError('Unknown search or heuristic')
    if config['search'] == 'rhc' and config['heuristic'] != 'hcff':
        raise ValueError('Refinement search requires hcff')
    return {k: config[k] for k in ['domain', 'problem', 'search', 'heuristic']}


def trace_worker(config, output, limit=6000):
    from pyperplan.planner import _parse, _ground, find_domain, HEURISTICS, SEARCHES
    from pyperplan.search import searchspace
    from pyperplan.ordering import LeastFailedFirst
    from pyperplan.search.benchmarking import Benchmark
    count = 0
    started = time.monotonic()

    class TraceLimit(Exception):
        pass

    def emit(kind, **data):
        nonlocal count
        if count >= limit:
            raise TraceLimit()
        output.put(dict(kind=kind, time=round(time.monotonic()-started, 4), **data))
        count += 1

    try:
        path = str(ROOT / 'benchmarks' / config['domain'] / config['problem'])
        emit('phase', message='Parsing and grounding PDDL')
        task = _ground(_parse(find_domain(path), path))
        emit('task', name=task.name, facts=len(task.facts), operators=len(task.operators),
             goals=sorted(task.goals), initial=sorted(task.initial_state))
        nodes = {}
        retained = []  # Keep IDs stable even after the search discards nodes.
        by_state = {}
        def register(node):
            if id(node) not in nodes:
                parent = register(node.parent) if node.parent else None
                ident = len(nodes)
                nodes[id(node)] = ident
                retained.append(node)
                duplicate = by_state.get(node.state)
                by_state.setdefault(node.state, ident)
                emit('node', id=ident, parent=parent, depth=node.g,
                     action=node.action.name if node.action else 'Initial state',
                     facts=sorted(node.state), duplicate=duplicate,
                     goal=task.goal_reached(node.state))
            return nodes[id(node)]
        root_factory, child_factory = searchspace.make_root_node, searchspace.make_child_node
        def root(state):
            node = root_factory(state)
            register(node)
            return node
        def child(parent, op, state):
            node = child_factory(parent, op, state)
            register(node)
            return node
        searchspace.make_root_node, searchspace.make_child_node = root, child
        successors = task.get_successor_states
        def expand(state):
            emit('expand', id=by_state.get(state), facts=sorted(state))
            return successors(state)
        task.get_successor_states = expand
        ordered = LeastFailedFirst.successor_generator
        def ordered_expand(self, state):
            emit('expand', id=by_state.get(state), facts=sorted(state))
            yield from ordered(self, state)
        LeastFailedFirst.successor_generator = ordered_expand
        lookahead = Benchmark.log_lookahead
        restart = Benchmark.restart
        def traced_lookahead(self, success, expansions, calls, ordering_calls, message):
            emit('lookahead', success=success, expansions=expansions, calls=calls, message=message)
            return lookahead(self, success, expansions, calls, ordering_calls, message)
        def traced_restart(self):
            emit('restart', message='Search restarted or backtracked')
            return restart(self)
        Benchmark.log_lookahead = traced_lookahead
        Benchmark.restart = traced_restart
        inner = HEURISTICS[config['heuristic']](task)
        class Observed:
            name = config['heuristic']
            def __call__(self, node):
                ident = register(node)
                extra = {}
                if hasattr(inner, 'evaluate'):
                    result = inner.evaluate(node)
                    value = result.value
                    extra['relaxed'] = [op.name for op in result.plan[:40]] if result.plan is not None else None
                    extra['relaxed_length'] = len(result.plan) if result.plan is not None else None
                else:
                    value = inner(node)
                emit('evaluate', id=ident, h=value if value != float('inf') else None,
                     version=getattr(inner, 'version', 0), **extra)
                return value
            def refine(self, node):
                before = inner.graph.conjunctions
                changed = inner.refine(node)
                emit('refine', id=register(node), changed=changed,
                     conjunctions=[sorted(c) for c in inner.graph.conjunctions-before],
                     version=inner.version, growth=inner.counter_growth)
                return changed
        solution = SEARCHES[config['search']](task) if config['search'] == 'bfs' else SEARCHES[config['search']](task, Observed())
        if solution is not None:
            state = task.initial_state
            states = [sorted(state)]
            for op in solution:
                assert op.applicable(state), 'Invalid solution action'
                state = op.apply(state)
                states.append(sorted(state))
            assert task.goal_reached(state), 'Invalid final state'
            emit('solution', actions=[op.name for op in solution], states=states)
        output.put(dict(kind='done', status='solved' if solution is not None else 'no_plan', time=time.monotonic()-started))
    except TraceLimit:
        output.put(dict(kind='done', status='trace_limit', message='Trace capped at 6,000 events. This is not an unsolvability result.'))
    except Exception as exc:
        output.put(dict(kind='done', status='error', message=str(exc)))


class Runs:
    def __init__(self):
        self.lock = threading.Lock()
        self.job = None
    def stop(self):
        if self.job:
            process = self.job['process']
            if process.is_alive():
                process.terminate()
            process.join(timeout=2)
            self.job['queue'].close()
    def start(self, config):
        with self.lock:
            self.stop()
            ctx = mp.get_context('spawn')
            queue = ctx.Queue()
            process = ctx.Process(target=trace_worker, args=(config, queue), daemon=True)
            ident = uuid.uuid4().hex
            self.job = dict(id=ident, process=process, queue=queue, events=[], start=time.monotonic(), done=False)
            process.start()
            timer = threading.Timer(60, self.expire, args=(ident,))
            timer.daemon = True
            timer.start()
            return ident
    def expire(self, ident):
        with self.lock:
            if self.job and self.job['id'] == ident and not self.job['done']:
                if self.job['process'].is_alive():
                    self.job['process'].terminate()
                    self.job['events'].append(dict(kind='done', status='timeout', message='60-second exploration limit reached'))
                    self.job['done'] = True
    def read(self, ident, after):
        with self.lock:
            job = self.job
            if not job or job['id'] != ident:
                raise ValueError('Run no longer available')
            while not job['done']:
                try:
                    event = job['queue'].get_nowait()
                    job['events'].append(event)
                    if event['kind'] == 'done':
                        job['done'] = True
                except Empty:
                    break
            if not job['done'] and time.monotonic()-job['start'] > 60:
                job['process'].terminate()
                job['events'].append(dict(kind='done', status='timeout', message='60-second exploration limit reached'))
                job['done'] = True
            return dict(events=job['events'][after:], total=len(job['events']), done=job['done'])


class Handler(BaseHTTPRequestHandler):
    runs = Runs()
    def log_message(self, *args):
        pass
    def reply(self, data, status=200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)
    def do_GET(self):
        url = urlparse(self.path)
        try:
            if url.path == '/api/catalog':
                return self.reply(dict(domains=catalog(), searches=SEARCHES, heuristics=HEURISTICS))
            if url.path == '/api/events':
                query = parse_qs(url.query)
                return self.reply(self.runs.read(query['id'][0], max(0, int(query.get('after', ['0'])[0]))))
            files = {'/': ('index.html', 'text/html'), '/app.js': ('app.js', 'text/javascript'), '/style.css': ('style.css', 'text/css')}
            if url.path not in files:
                return self.reply({'error': 'Not found'}, 404)
            filename, mime = files[url.path]
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.end_headers()
            self.wfile.write((ROOT/'explorer'/filename).read_bytes())
        except (ValueError, KeyError) as exc:
            self.reply({'error': str(exc)}, 400)
    def do_POST(self):
        # Browser writes must come from this local origin; no CORS exposure.
        origin = self.headers.get('Origin')
        if origin and origin != 'http://' + self.headers.get('Host', ''):
            return self.reply({'error': 'Origin rejected'}, 403)
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 <= length <= 4096:
                raise ValueError('Request too large')
            config = json.loads(self.rfile.read(length) or b'{}')
            if self.path == '/api/run':
                return self.reply({'id': self.runs.start(validate_config(config))})
            if self.path == '/api/stop':
                with self.runs.lock:
                    self.runs.stop()
                    if self.runs.job:
                        self.runs.job['events'].append(dict(kind='done', status='cancelled'))
                        self.runs.job['done'] = True
                return self.reply({'ok': True})
            self.reply({'error': 'Not found'}, 404)
        except (ValueError, KeyError, TypeError) as exc:
            self.reply({'error': str(exc)}, 400)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    print('Planner explorer: http://127.0.0.1:%d' % args.port, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        Handler.runs.stop()
        server.server_close()


if __name__ == '__main__':
    main()
