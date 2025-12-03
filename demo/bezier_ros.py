"""
Backward-compatibility shim: re-export everything from the shared core.
Prefer importing directly from `bezier_core`.
"""

from bezier_core import *  # noqa: F401,F403
