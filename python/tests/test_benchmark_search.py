from _bench_import import search


def test_expand_count_and_dicts():
    spec = {"grid": {"a": [1, 2, 3], "b": [10, 20]}}
    combos = search.expand(spec)
    assert len(combos) == 6
    assert {"a": 1, "b": 10} in combos
    assert {"a": 3, "b": 20} in combos
    for c in combos:
        assert set(c) == {"a", "b"}


def test_expand_single_key():
    combos = search.expand({"grid": {"max_iter": [3, 4, 5]}})
    assert combos == [{"max_iter": 3}, {"max_iter": 4}, {"max_iter": 5}]


def test_expand_empty_grid():
    assert search.expand({"grid": {}}) == [{}]
    assert search.expand({}) == [{}]
