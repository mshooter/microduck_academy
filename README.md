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
* [x] Headless smoke test: stand, walk, save one head-camera frame
* [x] `docs/api.md`: the message contract
* [x] `duckd_sim.py`: the server (Unix socket, one JSON-RPC object per line)
* [ ] Install check on a second machine

## Install

Needs [uv](https://docs.astral.sh/uv/) (macOS: `brew install uv`).

```bash
git clone https://github.com/mshooter/microduck_academy.git
cd microduck_academy
uv sync
uv run pytest -q
```

uv fetches Python 3.12 if your machine lacks it.

## Run the server

```bash
uv run python -m microduck_academy.duckd_sim            # terminal 1: headless, listens on /tmp/duckd.sock
uv run python -m microduck_academy.duckd_sim --viewer   # same, with a MuJoCo window (on macOS: replace python with mjpython)
uv run python examples/client.py                         # terminal 2: hello, subscribe, walk 5 s, stop
```

The server speaks [`docs/api.md`](docs/api.md): the real robot's `robot.*` intents over a Unix
socket, one JSON-RPC object per line, plus `sim.frame`, `sim.reset` and `sim.spawnBall`.
`tests/test_contract.py` runs the example client against it.

Versions are pinned to Pollen's training repo (MuJoCo 3.10.0, onnxruntime 1.24.4,
numpy 2.4.1). On macOS, MuJoCo's viewer window needs `mjpython` instead of `python`;
headless use is unaffected.

## Layout

Read in this order if you are new: `docs/api.md` (the quick start at the top), `examples/client.py`,
then `duckd_sim.py`. Each file starts with a docstring saying what it is for.

| path | what it is |
|---|---|
| `docs/api.md` | The message contract: every method, its fields, and where each name comes from in Pollen's source |
| `src/microduck_academy/duckd_sim.py` | The server. Runs the simulation at 50 Hz in a thread and maps each protocol method onto Pollen's policy loop. Start here to add a method |
| `src/microduck_academy/rpc.py` | JSON-RPC over a Unix socket, one object per line. Knows nothing about ducks; reusable for any handler |
| `src/microduck_academy/smoke.py` | Model loading, the two halves of the control loop (policy and physics), pose and camera helpers. The server is built from these |
| `src/microduck_academy/vendor/infer_policy.py` | Pollen's policy loop, copied unmodified (see `NOTICE`). Never edit; override in code |
| `examples/client.py` | The smallest complete client, standard library only. What a HAL sits on top of |
| `assets/microduck/`, `assets/policies/` | The robot model and the nine ONNX policies, copied unmodified from Pollen (see `NOTICE`) |
| `tests/test_assets.py` | The vendored files are complete and have the shapes the code relies on |
| `tests/test_smoke.py` | The policies walk in our loop and the camera renders |
| `tests/test_contract.py` | The server and the example client agree with `docs/api.md`, over a real socket |

## Troubleshooting

### macOS: `mjpython` fails with `Library not loaded: @executable_path/../lib/libpython3.X.dylib`

Running `uv run mjpython ...` fails with an error like:

```
failed to dlopen path '.../.venv/bin/python': ... Library not loaded: @executable_path/../lib/libpython3.12.dylib
  Reason: tried: ... '.../.venv/bin/../lib/libpython3.12.dylib' (no such file), ...
```

This is because `mjpython` needs the shared `libpython` library, and it cannot find it next to the venv's
`python`. uv's managed Python does actually ship the shared library, but the venv doesn't link to it, so we need to do this by hand. 

To do this, copy the shared library into the
venv from the uv-managed Python install. From the repo root and with the venv activated, run:

```bash
cp "$(uv run python -c 'import sys; print(sys.base_prefix)')"/lib/libpython3.*.dylib .venv/lib/
```

You will need to run this after recreating `.venv` (e.g. if you delete `.venv` folder). See
[mujoco#1923](https://github.com/google-deepmind/mujoco/issues/1923) and
[uv#8953](https://github.com/astral-sh/uv/issues/8953).

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
