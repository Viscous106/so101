#!/usr/bin/env bash
# Run the repeatability benchmark on several joints back-to-back, then analyse every run with both cameras.
#   run_batch.sh shoulder_lift elbow_flex wrist_flex        (env: PAIRS=15 DELTA=45 THIRD_EXPOSURE=75 ...)
# Torque is held between joints (HOLD=1) so the arm cannot sag while the next run starts; the last run releases it.
# Analysis (CPU-heavy ECC) runs only after all motion is done, so the arm never sits under torque waiting for it.
set -u
cd "$(dirname "$0")"
PY=/home/machine/so101-yash/.venv/bin/python
PAIRS=${PAIRS:-15}; DELTA=${DELTA:-45}
LOG=run_batch_$(date +%Y%m%d_%H%M%S).log
joints=("$@"); dirs=()
for i in "${!joints[@]}"; do
  j=${joints[$i]}
  if [ "$i" -lt $((${#joints[@]} - 1)) ]; then export HOLD=1; else export HOLD=0; fi
  echo "=== $j $(date +%T) HOLD=$HOLD ===" | tee -a "$LOG"
  $PY collect_v2.py "$j" "$DELTA" "$PAIRS" comp 2>&1 | tee -a "$LOG" | grep -E "ABORTED|DONE|stalled|released"
  dirs+=("$(ls -d runs/${j}_comp_* | tail -1)")
done
$PY - <<'EOF' | tee -a "$LOG"
from bus import Bus, JOINTS
b = Bus(); print("after batch:", {j: dict(tq=b.read(j,"torque_en"), P=b.read(j,"P"), I=b.read(j,"I"), temp=b.read(j,"temp")) for j in JOINTS}); b.close()
EOF
for d in "${dirs[@]}"; do
  echo "--- analyze2 (wrist) $d ---" | tee -a "$LOG"
  $PY analyze2.py "$d" 2>&1 | tee -a "$LOG" | grep -E "per_step|stair_r2|failed|min_cc|reversal|all_arrivals"
  echo "--- analyze3 (third-person) $d ---" | tee -a "$LOG"
  $PY analyze3.py "$d" 2>&1 | tee -a "$LOG" | grep -E "per_step|stair_r2|noise_floor|failed|min_cc|reversal|all_arrivals"
done
echo "=== batch done $(date +%T) ===" | tee -a "$LOG"
