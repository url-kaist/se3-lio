"""Parameter-sweep benchmark layer for SE(3)-LIO.

`evaluate` and `search` are pure-numpy/stdlib and import no compiled binding,
so they can be unit-tested on the host. `registry` and `run` pull in the LIO
core lazily (only when a sweep actually executes).
"""
