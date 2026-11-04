# E08 post-processing integration

`../launch_mola_post_processing` defaults to v8. It performs synchronous
expanded-reference reconstruction, attaches recorded GNSS through MOLA's ROS bag
reader, strips roof returns, runs the exact E08 dense recipe, nominates geometric
endpoint correspondences inside operator-selected ranges, runs quality-gated
endpoint and narrow corrections, levels once, and exports an ENU localization map.
The original v7 implementation is available with `--workflow v7`; the existing
`--legacy-auto-endpoint-loop` selects its v6 mode. No installed dependency is edited.

Use the two commands in `../mapping_instruction_manual_v8.md`. Reconstruction
produces `maps/NAME_post_processing/reconstruction/overlap_review.html`. Open it in
a browser to select offline keyframe IDs or ROS timestamp ranges. It includes the
commands for exporting scans to a rawlog and inspecting them in `rawlog-viewer`.
The HTML is generated locally and requires no network or hosting.

Roof dimensions default to the tested front-centred mounting: 58 by 40 inches,
at most four inches below the LiDAR. `--roof-length`, `--roof-width` and
`--roof-max-drop` accept metres. Use `--no-roof-filter` for an explicitly unfiltered
comparison. The filter preserves poses and all point fields; the GNSS adapter
retains the installed bridge's recorded fix messages, covariance, timestamps and
antenna transform. Attachment uses a strict one-second nearest-observation gate.

## Artifacts and failure handling

The requested offline SimpleMap, leveled SimpleMap and localization map keep their
v7 names. Configurations, command identities, timings, reconstruction health,
endpoint nominations and native-loader checks are retained in
`maps/NAME_post_processing/`. The raw reconstructed, roof-filtered and E08-corrected
maps remain available there. Optional seed, unleveled and georeference files are
removed only after success unless their existing save options are supplied.
Graph trajectories are retained to reproduce E08; `--save-trajectory-files` also
enables covariance sidecars. The three closure calls rebuild their own graphs;
cross-stage loop-factor persistence remains a future improvement.

Persistent text logs are `NAME_roof_filter.log`, `NAME_lc_e08.log`,
`NAME_lc_seed.log` and `NAME_lc_retroactive.log`. Final `.mm` publication requires
successful native loading, the expected keyframe count, a matching geographic
anchor and an identity ENU transform. Failed exports stay in the diagnostic
directory and do not replace the requested final map.

Reconstruction screens require at least 98% scan coverage and bag duration,
finite output, at least 95% of scans at ICP quality >=0.5, no below-0.5 streak
longer than five seconds, and no pose step over eight metres. Small startup scan
omissions observed in the original E08 recordings are tolerated. Reusing a
SimpleMap requires its original `trace.csv` and adjacent `replay.log`; supply
`--reconstruction-trace PATH` when these are not in the standard run directory or
beside the input map. Keyframe timestamps must also agree with the trace interval
and reach within ten seconds of its tail. These are health screens, not accuracy
measurements.

Each correction stage requires positive registration quality and final GNC
inliers; endpoint seeding additionally requires atomic consensus and at least
four final inliers. Nomination uses nearest corrected geometry with a default
1.5 m distance limit, rejects duplicated/out-of-order matches, and respects
`--loop-traversal`. `--endpoint-max-distance` changes that nomination limit; it
does not bypass geometric quality or consensus checks.

Existing output paths are refused unless `--allow-existing-output` is explicit.
Input/output aliases and outputs inside the original bag are always refused.
`--dry-run` performs read-only preflight and prints the plan without creating
output directories, building tools, replaying observations or generating maps.

## Dependency provenance

The tested loop-closure implementation is upstream commit
`cc6f30fdc2acb98315c0a10048d2fd40ccc55011` plus
[`e08_loop_closure.patch`](e08_loop_closure.patch), SHA256
`9ecd7ffae082dec3e4c4595daa25f8912f4024c12d78cad841cb1081dea05bb6`.
The patch is preserved evidence from the original experiments, not an automatically
applied modification. To reproduce it, use an isolated checkout of that revision,
review and apply the patch there, and build with the repository's normal KISS and
GTSAM dependencies. The unpatched single-frame adaptation is not equivalent E08.

Preflight checks the actual library resolved by `ldd` for the required submap,
ground-filter/voxel, stride, consensus and KISS features. Every run records the
executable and loaded-library SHA256 hashes; actual closure logs must confirm KISS
was enabled. Feature detection does not establish numerical equivalence to an
arbitrary future implementation. `tests/data/e08_dense.yaml` is the retained
Death Valley recipe and is compared in full against generated dense settings.

The GNSS and roof helpers build locally in `$DEV_WS/build/post_processing_tools`
(default workspace `/root/dev_ws`), using installed MRPT and MOLA libraries.
Nothing is installed over those dependencies.

## Verification

```bash
source /root/dev_ws/install/setup.bash
cmake -S scripts/post_processing -B /root/dev_ws/build/post_processing_tools
cmake --build /root/dev_ws/build/post_processing_tools -j2
python3 -m unittest discover -s scripts/post_processing/tests -v
bash -n scripts/launch_mola_post_processing
```

The native test uses six synthetic keyframes and a three-fix synthetic bag. It
checks covariance and antenna TF preservation, roof-point removal, the timestamp
gate, and retained GNSS serialization. It does not run odometry. Integration has
been checked with fixtures and a complete dry-run; a new real-data map generation
and localization replay remain separate validation steps.
