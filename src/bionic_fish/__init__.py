"""Vision and control stack for the underwater pipe-inspection challenge.

The package is a refactor of the reference platform in ``reference/``
(see ``.dsh/agents/reference-pipeline.md``), retargeted from red-pipe tracking
and object retrieval to LED-strip tracking and leak-marker reporting under the
rules in ``docs/``.

Module map::

    config.py    every deployment-varying number, loaded from YAML
    arena.py     synthetic pool and nadir camera (the only data source we have)
    vision.py    strip centre line, dark-blob markers, start/finish tape
    reporting.py track association, the three report gates, the p > 3/8 decision
    control.py   cascaded lateral control and the speed schedule
    fsm.py       state machine and the four hard termination conditions
    platform.py  MAVLink link, annunciator (light, buzzer, speech)
    runtime.py   assembly and the main loop
"""

from __future__ import annotations

__all__ = ["__version__"]

__version__ = "0.1.0"
