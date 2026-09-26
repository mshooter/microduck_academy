# microduck_academy

[![CI](https://github.com/mshooter/microduck_academy/actions/workflows/ci.yml/badge.svg)](https://github.com/mshooter/microduck_academy/actions/workflows/ci.yml)

A MuJoCo simulation server for Pollen Robotics' [Microduck](https://github.com/pollen-robotics/microduck),
under construction. When done it will run the duck's shipped policies and speak the robot's own
intent API (`robot.move`, `robot.do`, `robot.look`, `robot.state`), so a client written
against it should later work on a real duck unchanged.

First stage of a [JdeRobot Robotics Academy](https://jderobot.github.io/RoboticsAcademy/)
exercise, see [RoboticsInfrastructure#807](https://github.com/JdeRobot/RoboticsInfrastructure/issues/807).
Stage 1 is this standalone MuJoCo demo; stage 2 is Gazebo; stage 3 is Academy integration.

## Status

* [x] Vendored robot model and the nine policies, with `NOTICE`
* [x] Tests and CI (Ubuntu and macOS)
* [ ] Headless smoke test: stand, walk, save one head-camera frame
* [ ] `docs/api.md`: the message contract
* [ ] `duckd_sim.py`: the server (Unix socket, one JSON-RPC object per line)
* [ ] Install check on a second machine

## Install

Needs [uv](https://docs.astral.sh/uv/) (macOS: `brew install uv`).

```bash
git clone https://github.com/mshooter/microduck_academy.git
cd microduck_academy
uv sync
uv run pytest -q
```

There is nothing to run yet beyond the tests. uv fetches Python 3.12 if your machine lacks it.

Versions are pinned to Pollen's training repo (MuJoCo 3.10.0, onnxruntime 1.24.4,
numpy 2.4.1). On macOS, MuJoCo's viewer window needs `mjpython` instead of `python`;
headless use is unaffected.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for setup, the pull request flow, and the rules that
are not obvious (vendored files, read-only policies, where message names come from).

## People

See [CONTRIBUTORS.md](CONTRIBUTORS.md).

## Upstream

* Model and policies: [pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl)
  and [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck), commits in [NOTICE](NOTICE).
* Message names: [`duck-ipc-proto`](https://github.com/pollen-robotics/microduck/tree/main/duck-ipc-proto)
  in the same repo.

## Licence

[Apache 2.0](LICENSE). The vendored model and policies are Pollen Robotics', also Apache 2.0; see [NOTICE](NOTICE).
