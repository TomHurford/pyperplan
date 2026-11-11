import multiprocessing as mp
from queue import Empty

import pytest

from explorer.server import SEARCHES, trace_worker, validate_config


@pytest.mark.parametrize('search', SEARCHES)
def test_trace_reports_valid_solution(search):
    config = dict(domain='rovers', problem='task01.pddl', search=search, heuristic='hcff')
    context = mp.get_context('spawn')
    queue = context.Queue()
    process = context.Process(target=trace_worker, args=(config, queue))
    process.start()
    events = []
    try:
        while True:
            event = queue.get(timeout=15)
            events.append(event)
            if event['kind'] == 'done':
                break
        assert events[-1]['status'] == 'solved', events[-1]
        assert any(e['kind'] == 'expand' for e in events)
        solution = next(e for e in events if e['kind'] == 'solution')
        assert len(solution['states']) == len(solution['actions'])+1
        nodes = [e for e in events if e['kind'] == 'node']
        assert [e['id'] for e in nodes] == list(range(len(nodes)))
    finally:
        process.join(timeout=2)
        if process.is_alive():
            process.terminate()
            process.join()
        queue.close()


def test_rejects_unlisted_paths_and_incompatible_search():
    with pytest.raises(ValueError):
        validate_config(dict(domain='../', problem='secret', search='bfs', heuristic='hff'))
    with pytest.raises(ValueError):
        validate_config(dict(domain='rovers', problem='task01.pddl', search='rhc', heuristic='hff'))
