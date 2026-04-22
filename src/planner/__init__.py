"""
Planner package: hybrid A* search and path post-processing.

Public API:
    hybrid_astar  — main planning entry point
    HybridConfig  — search parameter dataclass
    PlanResult    — return type carrying the path and debug info
"""

from planner.astar import hybrid_astar, HybridConfig, PlanResult
