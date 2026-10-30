# Offensive simulation evidence

These are desktop simulation observations, not hardware or physical scoring
qualification. The match observer selects `AutoDrive offensive` and holds left
bumper in teleop. It does not supply goals, collection, aim, or feed commands.
The field fuel grid, cameras, game-piece contacts and hub scoring are modeled
approximations; estimated possession is not a measured hopper sensor.

The candidate was `e84cbbd` in the policy worktree, based on season commit
`a77cf96`, plus the observed-FUEL interface. All matches used a fresh process,
the configured field colliders, a 20/3/140 second practice sequence, and the
field-aware desktop binary. The observer connects to its task-owned local NT
and HAL websocket endpoints. HAL websocket listens on all interfaces.

| Trial | Package and executable SHA-256 | Result |
|---|---|---|
| [Blue, R inactive first](offensive-public-blue.json.gz) | Public MecoBase 2026.2.10, `de868a2a5912ddcac9fbd12b5c3608f2cf85a0be151a4fe879fd6f824ea0a986` | Complete, 7 auto scores, 92 teleop pickups, 35 teleop scores, stopped on disable; 54 stored at completion. |
| [Red, R inactive first, debugger](offensive-staged-red-gdb.json.gz) | Local staged MecoBase 2026.2.10, `dc109e1860b1cef28de2b445cf15e88a2c5daade2fb6ac0f38fa2dc394ccd1aa` | Complete, 8 auto scores, 54 teleop pickups, 43 teleop scores, stopped on disable. No `std::bad_optional_access` caught. |
| [Red, R inactive first, repeat](offensive-staged-red-repeat.json.gz) | Same staged executable as above | Complete, 9 auto scores, 55 teleop pickups, 46 teleop scores, stopped on disable. |

The public Linux x86-64 debug MecoBase archive downloaded from the published
Maven URL and Gradle's cached archive both had SHA-256
`6307ff7159a04fbb910be58326f42bbd2f83f6ab82eee959ba562b3f1f53e0ab`.
The local staged archive differed (`22c535a69a48861eabdebc339c31dc6fe1c4628d7e86c15d6ffe463a2171f66e`),
so the two builds must not be treated as identical. Public build command:

```sh
JAVA_HOME=/home/brian/wpilib/2026/jdk ./gradlew installFrcUserProgramLinuxx86-64DebugExecutable --max-workers=2
```

The matching desktop debug HAL websocket extension and installed debug binary
were run as a task-owned snapshot, with `FRC_SIM_NT_PORT=5811` and
`HALSIMWS_PORT=3301`. Example observer command:

```sh
python scripts/autodrive_offensive_match.py --alliance blue --auto-result R --output artifacts/offensive-public-blue.json
```

[Public](offensive-public-safety.json) and [staged](offensive-staged-safety.json)
probes injected invalid game data while the command was active, restored it,
released the held command, and disabled after re-engagement. Each observed
zero requested drive speed and zero bottom-indexer, top-indexer and conveyor
voltage after the relevant stop. The staged probe interrupted an active drive;
the public probe began in score phase with feed already off. Neither proves
interruption during an actively feeding volley or checks every motor.

## Unresolved failure

One earlier red/R run of the staged executable aborted at about 160.5 seconds
of a 163-second practice match. The last live observation was 49 total scores
at roughly 8 seconds remaining. The process reported `bad optional access`
followed by termination from `std::bad_optional_access`; no stack or core was
retained. The original observer then lost its partial trace when websocket
cleanup raised. That cleanup path was corrected before the repeats above.
The subsequent debugger and non-debugger repeats passed, but neither identifies
or fixes the exception's cause. The original process log was retained at
`/tmp/offense-sim-red.log` on the validation host and may be transient.

A separate staged blue trace showed the robot remaining near the modeled bump
at approximately (4.89, 2.18) m while requesting motion late in the match.
It stopped at disable. One successful trench return is not proof of reliable
terrain traversal or recovery from field contact.
