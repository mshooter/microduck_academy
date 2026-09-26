# Duck server API

The message contract between the duck server (`duckd_sim.py`, MuJoCo) and any
client, such as a Robotics Academy `HAL.py`. Every `robot.*` name, parameter
and state field is copied from Pollen Robotics' real robot, so a client
written against this server should later work on a physical Microduck.
Anything prefixed `sim.` exists only in this simulator.

Status: draft, 2026-09-26. The server is not written yet; section 5 lists
what is still to be confirmed. The appendix says where every name comes from.

## Quick start

Five things cover a fetch exercise. Each is one line of JSON on a Unix
socket; `→` is what you send, `←` is what comes back.

**1. Connect and say hello.** The socket is `/tmp/duckd.sock` by default.

```text
→ {"jsonrpc":"2.0","id":1,"method":"hello","params":{"api_version":16}}
← {"jsonrpc":"2.0","id":1,"result":{"api_version":16,"daemon_version":null,"revision":null}}
```

**2. Ask for state updates.** From now on the server sends a `robot.state`
line at the rate you asked for.

```text
→ {"jsonrpc":"2.0","id":2,"method":"robot.subscribe","params":{"hz":10}}
← {"jsonrpc":"2.0","id":2,"result":{"accepted":true,"walk":"walk.onnx","stand":"stand.onnx"}}
← {"jsonrpc":"2.0","method":"robot.state","params":{"t":1.0,"policy":"walk","safety":{"fallen":false,...},"odom":{"position":[0.63,0.05,0.17],"yaw":-0.19},...}}
```

**3. Walk.** Send the velocity you want, and keep sending it, about 20
times a second; the server forgets it after a moment, like a released
joystick. Below about 0.25 m/s the duck does not move.

```text
→ {"jsonrpc":"2.0","method":"robot.move","params":{"vx":0.3,"vy":0.0,"vyaw":0.0}}
```

**4. Look at the floor.** With the head level the camera cannot see closer
than about half a metre ahead.

```text
→ {"jsonrpc":"2.0","id":3,"method":"robot.look","params":{"x":0.3,"y":0.0,"z":-0.12}}
← {"jsonrpc":"2.0","id":3,"result":{"head":{...},"clamped":false}}
```

**5. Get a camera image.** A JPEG, 640 by 360, base64 in the reply.

```text
→ {"jsonrpc":"2.0","id":4,"method":"sim.frame"}
← {"jsonrpc":"2.0","id":4,"result":{"width":640,"height":360,"format":"jpeg","data":"<base64>"}}
```

In Python, using the client in `examples/client.py`:

```python
import time
from examples.client import DuckClient          # run from the repo root

duck = DuckClient("/tmp/duckd.sock")
duck.call("hello", {"api_version": 16})
duck.call("robot.subscribe", {"hz": 10})

for _ in range(100):                                   # 5 seconds at 20 Hz
    duck.notify("robot.move", {"vx": 0.3, "vy": 0.0, "vyaw": 0.0})
    state = duck.state()
    if state and state["safety"]["fallen"]:            # note: nested under "safety"
        break
    time.sleep(0.05)

duck.call("robot.stop")
```

That is the whole loop. The rest of this document is the reference.

## 1. How messages work

**Transport.** One JSON object per line over a Unix domain socket. A Unix
socket is a file two programs on the same machine read and write to talk.
The real robot listens on `/run/robotd.sock`; the simulator on
`/tmp/duckd.sock` (a command-line option), because `/run` needs root.

**JSON-RPC 2.0.** Every message has `jsonrpc: "2.0"` and a `method`.
Arguments go in `params`. If you include an `id`, you get a reply with the
same `id` carrying `result` or `error`. If you leave `id` out, the message
is a *notification*: nobody replies. Replies are matched by `id`, not by
order, so other lines may arrive in between. Pollen's own example:

```text
→ {"jsonrpc":"2.0","id":1,"method":"update.apply","params":{...}}
← {"jsonrpc":"2.0","method":"update.progress","params":{...}}   (no id)
← {"jsonrpc":"2.0","method":"update.progress","params":{...}}
← {"jsonrpc":"2.0","id":1,"result":{...}}
```

**Two kinds of intent.** This decides whether you wait for an answer.

| kind | sent as | methods | why |
|---|---|---|---|
| continuous | notification, resent 20 to 50 Hz, last one wins, expires | `robot.move`, `robot.head` | a stale command is worse than none, so nothing is queued or acknowledged |
| discrete | request with `id`, answered | `robot.stop`, `robot.look`, `robot.do`, `robot.subscribe`, all `sim.*` | the caller needs to know whether it was accepted and why not |

**Refusal is not an error.** A discrete intent can answer
`{"accepted": false, "reason": "..."}`, for example when safety will not
enable a policy on a fallen robot. Check `accepted`; do not only check for
`error`.

**Errors** replace `result` with `error: {"code", "message", "data"?}`.
The codes are the JSON-RPC standard ones: `-32601` unknown method,
`-32602` bad parameters (a misspelt skill name, for example).

**Handshake.** Send `hello` with `api_version: 16` once after connecting.
The reply's `daemon_version` and `revision` are `null` on the simulator
and on a laptop build of the real daemon.

**Units and frame.** Metres, radians, seconds. `x` forward, `y` left, `z`
up, positive yaw turns left. This is the ROS convention, and Robotics
Academy's `setW` sign, so nothing is flipped anywhere.

## 2. Robot methods

The seven methods the simulator implements. All exist unchanged on the real
robot.

### robot.move

Velocity twist. Continuous: send as a notification, keep sending.

| field | type | meaning |
|---|---|---|
| `vx` | number | forward, m/s |
| `vy` | number | left, m/s (the duck can strafe) |
| `vyaw` | number | yaw rate, rad/s, positive turns left |

Omitted fields are 0; unknown fields are rejected.

```text
→ {"jsonrpc":"2.0","method":"robot.move","params":{"vx":0.3,"vy":0.0,"vyaw":0.0}}
```

Measured on the shipped walker (2026-09-26): `|vx|` below about 0.25 m/s
does nothing, the duck stands still. The useful range is about 0.25 to
0.4 m/s (the policy was trained on ±0.4). A student calling `setV(0.2)`
will see no motion. The velocity actually applied, after any clamping,
comes back in `robot.state` as `move.applied` with the reasons in
`move.limited_by`.

### robot.stop

Zero the velocity. Not "go limp". Discrete.

```text
→ {"jsonrpc":"2.0","id":2,"method":"robot.stop"}
← {"jsonrpc":"2.0","id":2,"result":{"accepted":true}}
```

`reason` appears only when `accepted` is false.

### robot.look

Point the camera at a point in the trunk frame; the server solves the head
joints. Discrete.

| field | type | meaning |
|---|---|---|
| `x` | number | metres forward of the trunk origin |
| `y` | number | metres left of it |
| `z` | number | metres above it. Trunk frame, not floor: the floor is about 0.12 m below |
| `neck_pitch` | number | radians, posture not aim; the solver holds it and aims around it. Default 0 |

```text
→ {"jsonrpc":"2.0","id":3,"method":"robot.look","params":{"x":0.3,"y":0.0,"z":-0.12}}
← {"jsonrpc":"2.0","id":3,"result":{"head":{"neck_pitch":0.0,"head_pitch":0.4,"head_yaw":0.0,"head_roll":0.0},"clamped":false}}
```

The reply's `head` is what was sent to the joints; resend it as
`robot.head` to hold the gaze. `clamped: true` means the point is beyond
the head's reach and the joints are the closest gaze, not a lock. The
numbers in the example are illustrative.

Why a student needs this: with the head level the camera does not see the
floor closer than about 0.5 m, so a ball at the duck's feet is visible only
after looking down.

### robot.head

Head joint targets, radians. Continuous. The joint-space form of
`robot.look`; use `robot.look` when you care about a direction, since which
way a positive `head_yaw` turns is not stated in the protocol.

| field | type |
|---|---|
| `neck_pitch` | number |
| `head_pitch` | number |
| `head_yaw` | number |
| `head_roll` | number |

```text
→ {"jsonrpc":"2.0","method":"robot.head","params":{"neck_pitch":0.0,"head_pitch":0.4,"head_yaw":0.0,"head_roll":0.0}}
```

### robot.do

Run a one-shot scripted skill, or toggle sit/stand. Discrete. A refusal
names the scripted move already holding the robot.

| `skill` | what happens | duration |
|---|---|---|
| `"ground_pick"` | phase-scripted pick from the ground | about 3 s |
| `"kick_left"` | left-leg kick, blind to any ball | about 0.5 s |
| `"kick_right"` | right-leg kick | about 0.5 s |
| `"sit_toggle"` | sit if standing, stand if sitting | |
| `"roulade"` | forward roll; a request during a roll chains another | about 1 s |

```text
→ {"jsonrpc":"2.0","id":4,"method":"robot.do","params":{"skill":"ground_pick"}}
← {"jsonrpc":"2.0","id":4,"result":{"accepted":true}}
```

`accepted: true` means the skill started, not that it finished. Watch
`robot.state.policy` to see when the walker is back in charge.

### robot.subscribe

Turn the connection into a stream of `robot.state` notifications. Discrete.

| field | type | meaning |
|---|---|---|
| `hz` | integer, optional | how often to send state; absent means every control tick (50 Hz) |

```text
→ {"jsonrpc":"2.0","id":5,"method":"robot.subscribe","params":{"hz":10}}
← {"jsonrpc":"2.0","id":5,"result":{"accepted":true,"walk":"walk.onnx","stand":"stand.onnx"}}
← {"jsonrpc":"2.0","method":"robot.state","params":{...}}   (repeats, no id)
```

The reply names the policy files loaded for the life of the process
(`walk`, `stand`, and the skills as `sitstand`, `ground_pick`,
`kick_left`, ...); a missing key means that skill is not available. File
names shown are illustrative.

### robot.state

The frame a HAL's `getPose3d()` and `isFallen()` read from. Sent as a
notification, never with an `id`. This example is Pollen's own test frame
with an illustrative `odom` added (x, y and yaw from one 5 s smoke run; z
not measured):

```json
{
  "t": 1.0,
  "move": {"requested": [0.3, 0, 0], "applied": [0.3, 0, 0]},
  "head": [0, 0, 0, 0],
  "policy": "walk",
  "safety": {"fallen": false, "limp": false},
  "loop": {"hz": 50.0, "missed": 0},
  "joints": [],
  "targets": [],
  "odom": {"position": [0.63, 0.05, 0.17], "yaw": -0.19}
}
```

| key | type | meaning |
|---|---|---|
| `t` | number | seconds since the server started; for ordering, not wall time |
| `move.requested` | `[vx, vy, vyaw]` | what the client last asked |
| `move.applied` | `[vx, vy, vyaw]` | what the policy was given after safety clamps |
| `move.limited_by` | list of strings | why they differ; absent when they do not |
| `head` | 4 numbers | head joint angles, radians (order: see section 5) |
| `policy` | string | which network drove this tick: `walk`, `stand`, or `held` when none |
| **`safety.fallen`** | bool | the robot is down. **Nested under `safety`, not top level** |
| `safety.limp` | bool | gains dropped so the robot yields |
| `safety.gravity` | 3 numbers | projected gravity in the trunk frame; upright is about `[0, 0, -1]` |
| `safety.gain` | integer or absent | servo P gain actually running |
| `loop.hz` | number | achieved control rate over the last window |
| `loop.missed` | integer | ticks that overran, cumulative |
| `joints` | 15 numbers | measured joint angles, radians, in `JOINT_NAMES` order |
| `targets` | 15 numbers | commanded joint angles, same order, so a viewer can show tracking error |
| `odom.position` | `[x, y, z]` | trunk position, metres, in the frame the robot woke up in; `z` is height above ground |
| `odom.yaw` | number | heading, radians |

`JOINT_NAMES`, in order: `left_hip_yaw, left_hip_roll, left_hip_pitch,
left_knee, left_ankle, neck_pitch, head_pitch, head_yaw, head_roll, mouth,
right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee, right_ankle`.

Two rules from the real robot: `odom` may be absent from an old robot's
frame and then means "at the origin"; and clients must ignore keys they do
not recognise, so the simulator may add keys without breaking a HAL.

```python
fallen = state["safety"]["fallen"]        # not state["fallen"]
x, y, z = state["odom"]["position"]
yaw = state["odom"]["yaw"]
```

## 3. Simulator-only methods

These do not exist on the real robot, which streams video through a
different service and has no reset button. A HAL that uses them needs a
different implementation of the same functions on hardware. All are
discrete. Their shapes are ours and may change until the server lands.

### sim.frame

One head-camera image.

```text
→ {"jsonrpc":"2.0","id":6,"method":"sim.frame"}
← {"jsonrpc":"2.0","id":6,"result":{"width":640,"height":360,"format":"jpeg","data":"<base64>"}}
```

`data` is a base64-encoded JPEG; `base64.b64decode` then `cv2.imdecode`
gives a BGR array 360 rows by 640 columns, the same layout Robotics
Academy's `getImage()` returns. The size matches the real duck's default
stream.

Measured on 2026-09-26: the simulated frame is forward and upright. The
real stream is not: the physical camera is mounted a quarter turn off and
the consumer rotates it, so a HAL for hardware must add that rotation.
Vertical field of view is 45 degrees in the simulator. With the head level
the floor is not visible closer than about 0.5 m ahead.

### sim.reset

Put the duck back at the origin, standing, and the ball at its start
position.

```text
→ {"jsonrpc":"2.0","id":7,"method":"sim.reset"}
← {"jsonrpc":"2.0","id":7,"result":{"accepted":true}}
```

### sim.spawnBall

Place the ball at `x, y` on the floor, world frame, metres.

```text
→ {"jsonrpc":"2.0","id":8,"method":"sim.spawnBall","params":{"x":0.5,"y":0.0}}
← {"jsonrpc":"2.0","id":8,"result":{"accepted":true}}
```

## 4. Real-robot methods not implemented here

The real robot has more `robot.*` methods. The simulator answers them with
error `-32601` (method not found), so a HAL notices instead of silently
doing nothing: `robot.enable`, `robot.init`, `robot.relax`, `robot.pose`,
`robot.mouth`, `robot.sound`, `robot.theremin`, `robot.chorale`,
`robot.shutdown`, `robot.mode`, `robot.setMode`, plus the `update.*`,
`net.*`, `system.*`, `pad.*` and `config.*` namespaces. `robot.pose` (body
height and lean) and `robot.mouth` are the ones most likely to be wanted
for a fetch exercise later.

## 5. Not confirmed yet

Settled when the server exists, or when the real daemon's source is read.

- How long the server keeps a `robot.move` before it expires. Resend at 20
  Hz and you are safe on both the simulator and the real robot.
- Whether the real robot requires `hello` before other calls. The
  simulator accepts it and does not require it.
- The order of the four numbers in `state.head`. Taken as `neck_pitch,
  head_pitch, head_yaw, head_roll` until checked.
- What `state.policy` says while a skill runs.
- Dead zones for `vy` (trained ±0.3) and `vyaw` (trained ±1.0); only `vx`
  has been measured.
- The real camera's field of view; the simulator uses 45 degrees vertical.

## 6. Example client

[`examples/client.py`](../examples/client.py) is the smallest complete
client: standard library only, connects, says hello, subscribes, walks for
a few seconds at 20 Hz, stops. It is deliberately wire-level, not
Academy-shaped; `setV()`, `getImage()` and friends belong in `HAL.py`, one
layer up. Read it as "what `hal_interfaces/motors.py` and `camera.py` do
for ROS, done for this socket". It cannot run until the server exists;
the server's first test will be running it.

## Appendix: where each name comes from

Everything above is transcribed from `duck-ipc-proto/src/lib.rs` in
[pollen-robotics/microduck](https://github.com/pollen-robotics/microduck/blob/590b986bd8c0d50ae02cb3ea2f59c463b6828168/duck-ipc-proto/src/lib.rs)
at commit `590b986` (tag `daemon-v0.10.0`). Line numbers refer to that
commit. `sim.*` methods have no source; they are ours.

| item | lines |
|---|---|
| wire format, framing | 9 to 12; example 14 to 19 |
| `API_VERSION = 16` | 164 |
| socket path `/run/robotd.sock` | 191 |
| version mismatch is not refused | 55 to 58 |
| `hello`, params, reply | 260; 1477; 2057 to 2064 |
| continuous vs discrete intents | 299 to 305 |
| units and frame | 1488 to 1490 |
| error object; standard codes | 1404 to 1410; 531 to 535 |
| `IntentResult` `{accepted, reason?}` | 2525 to 2535 |
| `robot.move`; `MoveParams` | 308; 1500 to 1508 |
| `robot.head`; `HeadParams` | 310; 1517 to 1522 |
| `robot.look`; `LookParams`; `LookResult`; "never has to know which way head_yaw turns" | 313; 1531 to 1539; 1543 to 1549; 1526 |
| `robot.stop` | 315 |
| `robot.do`; refusal wording; `DoParams`; `Skill` names; `snake_case`; typo is invalid params | 352; 350 to 351; 1693; 1675 to 1687; 1674; 1671 |
| `robot.subscribe`; `SubscribeParams`; `SubscribeResult` | 416; 1758 to 1761; 1775 to 1800 |
| `robot.state`; struct | 425; 2560 to 2590 |
| `t`; `move` rename; `head`; `policy`; `safety`; `loop` rename; `joints`; `targets` | 2563; 2564; 2566; 2568; 2569; 2570; 2573; 2575 |
| `MoveState` requested, applied, limited_by | 2627; 2628; 2631 |
| `OdomState` position, yaw | 2620; 2622 |
| `SafetyState` fallen, limp, gravity, gain | 2638; 2640; 2648; 2654 |
| `LoopState` hz, missed | 2660; 2662 |
| `JOINT_NAMES` | 239 to 255 |
| Pollen's state frame used as the example; missing `odom` means origin | 4580 to 4591 |
| clients ignore unknown keys | 423 to 424 |
| the full `robot.*` list | 296 to 425 |
| measured facts (dead zone, frame geometry) | this repo, `src/microduck_academy/smoke.py`, 2026-09-26 |
