#!/usr/bin/env bash
set -euo pipefail. 

usage() {
  cat <<'USAGE'
Usage:
  generate_endpoint_lc_pipeline.sh INPUT.simplemap BASE.yaml OUTPUT.yaml [WINDOW_PERCENT] [PAIR_COUNT]

Replaces manual_loop_constraints with endpoint pairs selected from keyframe
poses. Early poses are sampled across the first WINDOW_PERCENT; each is paired
with its nearest pose in the last WINDOW_PERCENT. WINDOW_PERCENT must be in
(0,5]. Defaults: 5.0 percent and 3 pairs.
USAGE
}

if [[ $# -lt 3 || $# -gt 5 ]]; then usage >&2; exit 2; fi

input_simplemap="$1"
base_pipeline="$2"
output_pipeline="$3"
window_percent="${4:-5.0}"
pair_count="${5:-3}"

[[ -f "$input_simplemap" ]] || { echo "SimpleMap not found: $input_simplemap" >&2; exit 1; }
[[ -f "$base_pipeline" ]] || { echo "Base pipeline not found: $base_pipeline" >&2; exit 1; }
awk -v p="$window_percent" 'BEGIN { exit !(p > 0.0 && p <= 5.0) }' || {
  echo "WINDOW_PERCENT must be greater than 0 and no more than 5.0: $window_percent" >&2
  exit 2
}
[[ "$pair_count" =~ ^[1-9][0-9]*$ ]] || {
  echo "PAIR_COUNT must be a positive integer: $pair_count" >&2
  exit 2
}

tmp_dir="$(mktemp -d /tmp/mola-endpoint-lc-XXXXXX)"
cleanup() { rm -rf -- "$tmp_dir"; }
trap cleanup EXIT

tum_file="$tmp_dir/keyframes.tum"
constraints_file="$tmp_dir/constraints.yaml"
sm-cli export-keyframes "$input_simplemap" --output "$tum_file"

awk -v pct="$window_percent" -v requested_pairs="$pair_count" '
  { ts[NR]=$1; x[NR]=$2; y[NR]=$3; z[NR]=$4 }
  END {
    n=NR
    if (n < 4) { print "Too few keyframes: " n > "/dev/stderr"; exit 3 }
    # Round down so the actual window never exceeds the requested percentage.
    window=int(n*pct/100.0); if (window<1) window=1
    pairs=requested_pairs; if (pairs>window) pairs=window
    late_first=n-window+1
    print "  manual_loop_constraints:"
    for (k=1; k<=pairs; k++) {
      early=int(k*(window+1)/(pairs+1)); if (early<1) early=1; if (early>window) early=window
      best_late=late_first; best_d2=-1
      for (j=late_first; j<=n; j++) {
        dx=x[early]-x[j]; dy=y[early]-y[j]; dz=z[early]-z[j]
        d2=dx*dx+dy*dy+dz*dz
        if (best_d2<0 || d2<best_d2) { best_d2=d2; best_late=j }
      }
      printf "    - timestamp_i: %.6f\n", ts[early]
      printf "      timestamp_j: %.6f\n", ts[best_late]
      print "      sigma_xyz: 0.20"
      print "      trust_as_inlier: false"
      printf "endpoint pair %d: early index %d, late index %d, initial distance %.3f m\n", k, early-1, best_late-1, sqrt(best_d2) > "/dev/stderr"
    }
    printf "selected %d pairs from %d/%d keyframes per endpoint window (%.3f%%)\n", pairs, window, n, 100.0*window/n > "/dev/stderr"
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
' "$base_pipeline" > "$output_pipeline"

echo "Generated endpoint loop-closure pipeline: $output_pipeline"
