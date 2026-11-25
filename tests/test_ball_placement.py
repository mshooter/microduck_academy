"""Tests for ball_placement: the seeded ball spot used by sim.reset."""

import pytest
from microduck_academy.duckd_sim import BALL_X, BALL_Y, ball_placement


def test_same_seed_same_placement():
    assert ball_placement(42) == ball_placement(42)


def test_different_seed_different_placement():
    assert ball_placement(1) != ball_placement(2)


def test_ball_placement_within_bounds():
    for seed in range(200):
        x, y = ball_placement(seed)
        assert BALL_X[0] <= x <= BALL_X[1], f"seed {seed}: x={x}"
        assert BALL_Y[0] <= y <= BALL_Y[1], f"seed {seed}: y={y}"


def test_seed_42_gives_its_published_placement():
    # Golden value: a seed's spot is public (exercises and graders quote seeds), so if this
    # fails, the maths changed and every published seed moved. Fix the code, not the numbers.
    # Recorded with:
    # uv run python -c "from microduck_academy.duckd_sim import ball_placement; print(ball_placement(42))"
    assert ball_placement(42) == pytest.approx(
        (1.1394267984578836, -0.47498924477733306)
    )
