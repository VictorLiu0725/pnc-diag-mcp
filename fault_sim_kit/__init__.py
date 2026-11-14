"""Fault injection and synthetic Golden MCAP dataset generator."""

from fault_sim_kit.generator import (
    BASE_STAMP_NS,
    DEFAULT_SCENARIOS,
    FaultScenarioConfig,
    generate_golden_dataset,
    simulate_scenario,
)

__all__ = [
    "BASE_STAMP_NS",
    "DEFAULT_SCENARIOS",
    "FaultScenarioConfig",
    "generate_golden_dataset",
    "simulate_scenario",
]
