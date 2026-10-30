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
| [Red, R inactive first, public Release](offensive-public-release-red.json.gz) | Public MecoBase 2026.2.10, `ed397709b0b098f13a7bda81d5956330804ece6f94bf5464a96e6794282ffcdc` | Complete, 108 teleop pickups, 45 teleop scores, stopped on disable. |
| [Red, R inactive first, patched PathPlanner](offensive-patched-pathplanner-red.json.gz) | Local PathPlanner `2026.1.2-meco1`, executable `4089237a833e15b7461a60f8114dc87b944ea6459d66c02b93753407827acb79` | Complete, 7 auto scores, 39 teleop pickups, 32 teleop scores, stopped on disable. |

The patched trial used the isolated PathPlanner fix at `6a64c40` and the
[recorded local dependency hashes](offensive-patched-pathplanner-hashes.sha256).
The replacement package is not published. Its fix addresses a separately
reproduced out-of-bounds defect; the earlier red crash had no stack, so its
cause remains unconfirmed.

The public Release red trial was the second practice in the same simulator
process. Its `complete` score sample of 11 predates the reset; autonomous and
teleop samples are after reset. The simulator reported quota errors writing its
WPILOG under `/tmp`; the NT observer trace was saved, but the WPILOG is incomplete.
The [Release library hashes](offensive-public-release-libs.sha256) record the
linked dependency files.

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
An additional [public Release probe](offensive-public-red-active-feed-safety.json)
injected missing game data while feeding was active and observed all three feed
voltages and the drive request go to zero. Its later release and disable checks
did not necessarily occur during feeding.

## Unresolved failure

One earlier red/R run of the staged executable aborted at about 160.5 seconds
of a 163-second practice match. The last live observation was 49 total scores
at roughly 8 seconds remaining. The process reported `bad optional access`
followed by termination from `std::bad_optional_access`; no stack or core was
retained. The original observer then lost its partial trace when websocket
cleanup raised. That cleanup path was corrected before the repeats above.
The subsequent debugger and non-debugger repeats passed, but neither identifies
or fixes the exception's cause. The [original process log](offensive-staged-red-failure.log.gz) is retained here.

A separate staged blue trace showed the robot remaining near the modeled bump
at approximately (4.89, 2.18) m while requesting motion late in the match.
It stopped at disable. One successful trench return is not proof of reliable
terrain traversal or recovery from field contact.

A further [blue match](offensive-blue-window-failure.json.gz) on desktop binary
`fa518239367bd6ee621f6f8bec9c9c106648141ea671fb60a276761db59af696`
scored six in autonomous and zero in teleop despite collecting 54 in teleop.
The shift-three return was interrupted by the hub changing inactive, and the
endgame return reached the shot phase with about two seconds left; no teleop
feed was observed. This is a failed trial, not proof of a single cause.
The same simulator's [active-feed interruption probe](offensive-active-feed-interruptions.json)
observed all three feed voltages above 1 V before match-data loss, release, and
disable respectively, and observed those outputs and drive requests stop
following each interruption. It does not establish physical hardware behavior.

A subsequent [blue timing-candidate match](offensive-blue-window-candidate.json.gz)
with the next-change route heuristic scored 7 in autonomous and 83 in teleop,
with 139 teleop pickups and a normal stop. Desktop binary SHA-256 was
`ce820a3e0fa31a3dc1be4f849f158306a70f6e3d917a95fff634e7ebf008c6b3`.
The full local build passed on desktop and roboRIO and all 30 desktop tests
passed in Debug and Release. This single run does not establish repeatability.
The same binary also completed a [red timing-candidate match](offensive-red-window-candidate.json.gz)
with 9 autonomous and 18 teleop scores, 21 teleop pickups, and a normal stop.

With the checked-in descriptor selecting public PathPlanner `2026.1.2-meco1`,
the full `JAVA_HOME=/home/brian/wpilib/2026/jdk ./gradlew build --max-workers=2`
built desktop and roboRIO and passed all 30 desktop tests in both configurations.
The Gradle-cached public headers, desktop debug/release and roboRIO debug/release
archives matched the public Maven downloads by SHA-256:

| Artifact | SHA-256 |
|---|---|
| headers | `72606f595bdf2ad9b2f32b1a645912bb6579739f933377b25d89b30b44466be3` |
| linuxx86-64 release | `32be3216378b3f65a7fee00b49f26ed972e0954a6e588c2a27362c247a2aadcd` |
| linuxx86-64 debug | `3bce52b31c5c8a3905c9a548ddb13c6866b1373764df7e29dd8ef2e9797d116b` |
| linuxathena release | `7c18c7faffed0172d2f051e776092d353afdac1ae00e9384c85c566596f9cfa4` |
| linuxathena debug | `aba5f9b6b9eb193383ced9129fc2379280cd68cfe0c1bb4facfa306234845fb9` |

The rebuilt desktop binary SHA-256 was
`6d4285ea749763964b8e13ba2deeb8e6514ee766b0514bb2cf991e05eeb425c2`.
A [blue match](offensive-published-pathplanner-blue.json.gz) scored 9 in auto
and 64 in teleop; a [red match](offensive-published-pathplanner-red.json.gz)
scored 16 in auto and 28 in teleop. Both completed and stopped normally. These
are desktop simulation trials, not hardware qualification or a reliability guarantee.
