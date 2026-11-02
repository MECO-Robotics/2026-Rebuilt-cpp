# Optimized mixed-hardware profile: laptop results

All eight cases passed. Each measured eight seconds of the full native desktop robot, with separate process-owned localhost endpoints and safe disable/cleanup. The baseline is this branch with the optimized profile disabled, not a physical Driver Station or radio measurement. Input events are spaced 73 ms apart; latency is from the scheduled event through input publication to the timestamped teleop drive command.

| Input | Control period | Extra NT traffic | Median ms | p95 ms | p99 ms |
|---|---:|---|---:|---:|---:|
| DS | 20 ms | none | 22.62 | 32.12 | 33.02 |
| DS | 20 ms | vision | 16.09 | 25.44 | 26.09 |
| NT | 20 ms | none | 13.08 | 23.19 | 24.29 |
| NT | 20 ms | vision | 14.44 | 22.87 | 23.54 |
| DS | 2 ms | none | 11.20 | 20.27 | 21.22 |
| DS | 2 ms | vision | 11.28 | 19.32 | 20.31 |
| NT | 2 ms | none | 3.44 | 6.21 | 7.13 |
| NT | 2 ms | vision | 4.19 | 7.85 | 10.13 |

## Individual changes

| Change | Baseline | Optimized | Evidence |
|---|---|---|---|
| Command scheduler | 50 Hz | 500 Hz nominal | Measured median command period ~2 ms; tails below |
| Gamepad publication | 50 Hz simulated DS/HAL | 200 Hz NT | Actual NT4 client to full robot command path |
| SPARK mechanism control | 50 Hz | ~99–100 Hz | Measured callback counters |
| Kraken mechanism control | 50 Hz | ~248–250 Hz on simulated FD | Measured callback counters; physical classic fallback 100 Hz |
| SPARK feedback | Vendor/persisted defaults | Requested 100 Hz | Configuration and compilation; physical delivery unmeasured |
| Kraken mechanism feedback/request resend | Defaults | Requested 250 Hz FD /100 Hz classic | Configuration and compilation; physical delivery unmeasured |
| Swerve odometry | Phoenix default 250 Hz FD /100 Hz classic | Requested 500 Hz FD /100 Hz classic | Configuration; physical delivery unmeasured |
| Detection NT batching | 100 ms | 5 ms request | Real factory/NT4 transport probe below |
| QuestNav NT receiver | Already 5 ms | Unchanged; consumed by faster scheduler | No headset/source-rate change claimed |
| Physics | 20 ms | 20 ms | Separate timestamp/counter gate; runtime/wall rate ~1.0 |

With extra traffic **none**, median reductions relative to that baseline were **42.2%** for NT input alone, **50.5%** for faster scheduling alone, and **84.8%** combined.
With extra traffic **vision**, median reductions relative to that baseline were **10.3%** for NT input alone, **29.9%** for faster scheduling alone, and **74.0%** combined.

Absolute values depend on source/robot phase and host scheduling; differences between load cases do not show that added traffic improves latency. The extra streams publish compact results at QuestNav-like 120 Hz, two gamepiece-camera-like 60 Hz streams and AprilTag-like 60 Hz. They do not run real inference.

## Scheduling limits

NT + fast control, traffic none: command interval median **1.998 ms**, p95 **4.286 ms**, p99 **4.684 ms**; physics runtime/wall ratio **0.99993**.
NT + fast control, traffic vision: command interval median **1.998 ms**, p95 **4.174 ms**, p99 **4.816 ms**; physics runtime/wall ratio **1.00001**.

This is a nominal 500 Hz profile, not a hard 2 ms deadline guarantee. The synchronous 20 ms physics/telemetry work and desktop OS scheduling create longer intervals followed by catch-up callbacks. Use actual-start/command intervals to judge timing. The fast-loop scheduled/lateness fields are approximate diagnostic bookkeeping, not measured kernel release deadlines.

## Perception transport probe

The actual `MakeNtObjectDetectionIO` factory was exercised over NT4 with a 60 Hz synthetic source for four seconds per profile. Baseline received 40/240 frames; optimized received 240/240: **6× more delivered frames**. Frame publication-to-read mean age was about 11 ms versus 6 ms in the initial probe. This is transport age, not camera exposure/inference latency. Producers must also configure low batching; `scripts/perception_publisher.py` supplies the matching 5 ms publisher for the existing protocol. It must be called by the actual coprocessor inference pipeline; no camera application was deployed.

## Build and reproduce

Two development worktrees are required: `../mecobase-fast-control` at public tag v2026.2.11 plus this patch, and `../mecosim-fast-control` at public tag v2026.0.16. The latter has no source edits: its adapter must be rebuilt against the development Base headers because the public configuration layout changed. The packaging script retains unchanged published Jolt objects, replaces all three adapter objects, checks membership, and uses hashed development coordinates. Public vendordeps/releases are not overwritten.

```sh
scripts/build_fast_profile.sh
/home/brian/.cache/rebuilt-latency-venv/bin/python scripts/frequency_matrix.py --seconds 8 --include-vision-load
```

Enable both `FRC_FAST_CONTROL=1` and `FRC_SIM_NT_GAMEPAD=1` for optimized simulation. Without the compile-time fast API flag, selecting fast mode fails explicitly. The standard public-dependency build remains a baseline build. The native development packages are not roboRIO/Windows releases or a hardware deployment.

Framework tests: 138 passed and one acceptance fixture skipped. Consumer safety/transport tests: three passed. Physical CAN utilization, delivered signal age, hardware trajectories, USB controller sampling, radios and mechanical response remain unmeasured. Preserve the Pigeon and existing bus topology; no speculative split-bus or gyro replacement was included.

Full records: `artifacts/frequency-matrix/20261102T070138Z/matrix-summary.json` and the eight case traces.

Validated raw matrix records and dependency provenance are retained in `artifacts/frequency-matrix-validated.zip`; unpack it at the repository root to restore the trace paths above. Failed startup trials remain only in the ignored local run directory.

The final loaded NT+fast smoke run after the follower-frame protection passed all gates; see `artifacts/fast-final-smoke.json`. The follower guard changes real Talon configuration and does not change the simulated input path used for the matrix.

Source commits: the MECO Base support is committed on `codex/fast-control-io` at `339d965`; the simulation adapter source remains the unmodified public v2026.0.16 tag, rebuilt against those headers. Native package provenance includes the exact pre-commit source patch that was tested.
