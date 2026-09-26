# Contributing

Thanks for looking. This repo is small and early, so the rules are short.

## Set up

```bash
git clone git@github.com:mshooter/microduck_academy.git
cd microduck_academy
uv sync
uv run pytest -q
```

## Making a change

1. Open an issue first if the change is more than a fix, so we can agree the shape.
2. Work on a branch, open a pull request against `main`. Keep it to one topic.
3. CI must be green on Ubuntu and macOS before merge.
4. Add or update a test when behaviour changes. `tests/` is plain pytest.
5. Commit messages: a short subject line, then a body saying *why*. Decisions that shape the
   code (transport, message names, pinned versions) belong in the commit that introduces them.

## Rules that are not obvious

* **Vendored files.** Everything under `assets/` is copied unmodified from Pollen Robotics'
  repos. Do not edit them. To update, copy again from a newer commit and record the new hash
  in `NOTICE`.
* **Policies are read-only.** Never edit an `.onnx` file. If the duck misbehaves, the loop
  around the policy is the suspect, not the network.
* **Message names are copied, not invented.** They come from Pollen's `duck-ipc-proto`.
  Anything that only exists in simulation is prefixed `sim.` so it is obvious it is not on the
  real robot.
* **Physics and inference stay separate.** Stepping the simulator and running the policy are
  different functions with a fixed inference rate. That split is what gets ported to Gazebo.
* **Pinned versions.** MuJoCo, onnxruntime and numpy are pinned to Pollen's training repo on
  purpose. Bumping them is a deliberate change with a reason in the commit message.
* **Dependencies.** `uv add <pkg>` (or `uv add --dev <pkg>`), then commit `pyproject.toml`
  and `uv.lock` together. CI runs `uv sync --locked` and fails if they disagree.

## Where this is going

Stage 1 (this repo) is a standalone MuJoCo demo. Stage 2 moves to Gazebo. Stage 3 integrates
into JdeRobot's RoboticsInfrastructure and Robotics Academy, which have
[their own contribution rules](https://github.com/JdeRobot/RoboticsInfrastructure/blob/humble-devel/CONTRIBUTING.md).
Discussion happens on [RoboticsInfrastructure#807](https://github.com/JdeRobot/RoboticsInfrastructure/issues/807).

## Licence

By contributing you agree your work is released under the Apache License 2.0, the same as the
rest of the repo.
