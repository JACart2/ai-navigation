#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  generate_waypoint_lc_pipelines.sh INPUT.simplemap BASE.yaml \
    SEED_OUTPUT.yaml AUTO_OUTPUT.yaml START_RANGE END_RANGE \
    [PAIR_COUNT] [same|reverse] [keyframes|timestamps]

START_RANGE and END_RANGE describe the two traversals in travel order. In
keyframes mode they are inclusive offline SimpleMap keyframe IDs (FIRST:LAST).
In timestamps mode they are absolute ROS timestamps in decimal seconds
(FIRST:LAST), as observed from /clock or a sensor header while replaying the
bag. Timestamps are resolved to the nearest offline SimpleMap keyframes.
PAIR_COUNT defaults to 6. Traversal direction defaults to reverse. The
automatic pipeline has no manual constraints and is intended for the narrow
retroactive second pass.
USAGE
}

if [[ $# -lt 6 || $# -gt 9 ]]; then usage >&2; exit 2; fi

input_simplemap="$1"
base_pipeline="$2"
seed_output="$3"
auto_output="$4"
start_range="$5"
end_range="$6"
pair_count="${7:-6}"
traversal="${8:-reverse}"
range_mode="${9:-keyframes}"

[[ -f "$input_simplemap" ]] || { echo "SimpleMap not found: $input_simplemap" >&2; exit 1; }
[[ -f "$base_pipeline" ]] || { echo "Base pipeline not found: $base_pipeline" >&2; exit 1; }
[[ "$pair_count" =~ ^[0-9]+$ ]] && (( pair_count >= 2 )) || { echo "PAIR_COUNT must be an integer of at least 2: $pair_count" >&2; exit 2; }
[[ "$traversal" == same || "$traversal" == reverse ]] || { echo "Traversal must be same or reverse: $traversal" >&2; exit 2; }
[[ "$range_mode" == keyframes || "$range_mode" == timestamps ]] || { echo "Range mode must be keyframes or timestamps: $range_mode" >&2; exit 2; }

parse_range() {
  local value="$1" label="$2"
  if [[ "$range_mode" == keyframes ]]; then
    [[ "$value" =~ ^([0-9]+):([0-9]+)$ ]] || { echo "$label must be FIRST:LAST using offline keyframe IDs: $value" >&2; exit 2; }
  else
    [[ "$value" =~ ^([0-9]+([.][0-9]+)?):([0-9]+([.][0-9]+)?)$ ]] || { echo "$label must be FIRST:LAST using absolute ROS seconds: $value" >&2; exit 2; }
  fi
  local first="${BASH_REMATCH[1]}" last="${BASH_REMATCH[3]:-${BASH_REMATCH[2]}}"
  awk -v a="$first" -v b="$last" 'BEGIN { exit !(a < b) }' || { echo "$label must be increasing: $value" >&2; exit 2; }
  printf '%s %s\n' "$first" "$last"
}

read -r start_first start_last < <(parse_range "$start_range" START_RANGE)
read -r end_first end_last < <(parse_range "$end_range" END_RANGE)

tmp_dir="$(mktemp -d /tmp/mola-waypoint-lc-XXXXXX)"
cleanup() { rm -rf -- "$tmp_dir"; }
trap cleanup EXIT

tum_file="$tmp_dir/keyframes.tum"
constraints_file="$tmp_dir/constraints.yaml"
sm-cli export-keyframes "$input_simplemap" --output "$tum_file"

awk -v sf="$start_first" -v sl="$start_last" -v ef="$end_first" -v el="$end_last" \
  -v pairs="$pair_count" -v traversal="$traversal" -v mode="$range_mode" '
  function nearest_offline(target,    q,best,bestd,d) {
    best=0; bestd=-1
    for (q=0; q<n; q++) {
      d=ts[q]-target; if (d<0) d=-d
      if (bestd<0 || d<bestd) { best=q; bestd=d }
    }
    return best
  }
  { ts[NR-1]=$1; x[NR-1]=$2; y[NR-1]=$3; z[NR-1]=$4 }
  END {
    n=NR
    if (n < 4) { print "Too few offline keyframes" > "/dev/stderr"; exit 3 }
    if (mode == "timestamps") {
      sf=nearest_offline(sf); sl=nearest_offline(sl)
      ef=nearest_offline(ef); el=nearest_offline(el)
      if (sf >= sl || ef >= el) { print "Timestamp ranges resolve to invalid offline keyframe ranges" > "/dev/stderr"; exit 3 }
    }
    if (sl >= n || el >= n) { printf "Range exceeds offline map with %d keyframes\n", n > "/dev/stderr"; exit 3 }
    if (pairs > sl-sf+1 || pairs > el-ef+1) { print "PAIR_COUNT exceeds one of the resolved keyframe ranges" > "/dev/stderr"; exit 3 }
    print "  manual_loop_constraints:"
    for (k=0; k<pairs; k++) {
      alpha=k/(pairs-1)
      if (traversal == "reverse") a=sl-alpha*(sl-sf)
      else a=sf+alpha*(sl-sf)
      b=ef+alpha*(el-ef)
      i=int(a+0.5); j=int(b+0.5); wi=i; wj=j
      dx=x[i]-x[j]; dy=y[i]-y[j]; dz=z[i]-z[j]
      sigma=0.20+0.20*alpha
      printf "    - timestamp_i: %.6f\n", ts[i]
      printf "      timestamp_j: %.6f\n", ts[j]
      printf "      sigma_xyz: %.3f\n", sigma
      print "      trust_as_inlier: false"
      printf "waypoint pair %d: selected %.6f <-> %.6f, offline frames %d <-> %d, raw distance %.3f m, sigma %.3f m\n", k+1, wi, wj, i, j, sqrt(dx*dx+dy*dy+dz*dz), sigma > "/dev/stderr"
    }
    printf "selected %d %s-direction pairs; %s inputs resolved to offline ranges %.0f:%.0f and %.0f:%.0f\n", pairs, traversal, mode, sf, sl, ef, el > "/dev/stderr"
  }
' "$tum_file" > "$constraints_file"

awk -v constraints="$constraints_file" '
  BEGIN { replacing=0; replaced=0 }
  /^  manual_loop_constraints:[[:space:]]*$/ {
    while ((getline line < constraints)>0) print line
    close(constraints); replacing=1; replaced=1; next
  }
  replacing && /^  # Optional: force loop closure edges/ { replacing=0 }
  !replacing { print }
  END { if (!replaced) { print "Base pipeline has no manual_loop_constraints block" > "/dev/stderr"; exit 4 } }
' "$base_pipeline" > "$seed_output"

awk '
  BEGIN { replacing=0; replaced=0 }
  /^  manual_loop_constraints:[[:space:]]*$/ { replacing=1; replaced=1; next }
  replacing && /^  # Optional: force loop closure edges/ { replacing=0 }
  !replacing { print }
  END { if (!replaced) { print "Base pipeline has no manual_loop_constraints block" > "/dev/stderr"; exit 4 } }
' "$base_pipeline" > "$auto_output"

echo "Generated waypoint seed pipeline: $seed_output"
echo "Generated automatic retroactive pipeline: $auto_output"
