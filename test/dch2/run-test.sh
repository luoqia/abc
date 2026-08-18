#!/usr/bin/env bash
# Permanent DCH2 regression battery.
# usage: run-test.sh <abc-binary>
#
# Covers: command registration/help, every option and invalid value, the
# post--j parser regression, deterministic j1/j4 identity, timeout/fault
# sensitivity, candidate/proved/rejected/conflict counter invariants,
# CHECK, post-DCH and post-mapping CEC, and default-off absence of test
# hooks. Fixtures are source-owned under the same directory.
set -u
ABC=$(readlink -f "${1:?usage: run-test.sh <abc-binary>}")
DIR=$(cd "$(dirname "$0")" && pwd)
SCRATCH=$(mktemp -d)
trap 'rm -rf "$SCRATCH"' EXIT
cd "$SCRATCH"

FAIL=0
ok()  { echo "PASS: $1"; }
bad() { echo "FAIL: $1"; FAIL=1; }
ts_hash() { tail -n +2 "$1" | sha256sum | awk '{print $1}'; }

run_abc() { # run_abc <tag> <script...>
  local tag=$1; shift
  "$ABC" -f /dev/stdin > "$tag.log" 2>&1 <<EOF
$*
EOF
  echo "$tag rc=$?"
}

# ---- 1. registration and help -------------------------------------
echo "== registration/help"
"$ABC" -c "help" > help_all.log 2>&1
grep -qE "^ dch2 " help_all.log && ok "dch2 listed in help" || bad "dch2 listed in help"
"$ABC" -c "dch2 -h" > help_dch2.log 2>&1
grep -q "usage: dch2" help_dch2.log && ok "dch2 -h usage" || bad "dch2 -h usage"
grep -qE "\-j <num>" help_dch2.log && ok "dch2 -h documents -j" || bad "dch2 -h documents -j"

# ---- 2. invalid values ----------------------------------------------
echo "== invalid values"
for args in "-j 0" "-W 9" "-H -1" "-C -1"; do
  run_abc "inv_$(echo $args | tr ' ' '_')" "read $DIR/add8.blif; strash; dch2 $args"
  if grep -qE "must be at least|cannot be negative|Invalid|usage: dch2" "inv_$(echo $args | tr ' ' '_').log"; then
    ok "dch2 $args rejected"
  else
    bad "dch2 $args rejected"
  fi
done

# ---- 3. every valid option (post--j parser regression included) -----
echo "== options"
run_abc opt_j2v  "read $DIR/add8.blif; strash; dch2 -j 2 -v"
grep -q "DCH2-BATCH" opt_j2v.log && ok "post--j -v parsed (verbose output present)" || bad "post--j -v parsed"
run_abc opt_j2c  "read $DIR/add8.blif; strash; dch2 -j 2 -C 100; write_blif opt_j2c.blif"
grep -q "DCH2: " opt_j2c.log && [ -s opt_j2c.blif ] && ok "post--j -C parsed (summary present)" || bad "post--j -C parsed"
run_abc opt_w10  "read $DIR/chain32.blif; strash; dch2 -W 10 -H 4 -S 7 -C 500 -j 1; write_blif opt_w10.blif"
grep -q "DCH2: " opt_w10.log && ok "combined options run (-W -H -S -C -j)" || bad "combined options run"

# ---- 4. j1/j4 identity and determinism --------------------------------
echo "== j1/j4 identity + determinism"
for fx in add8 dup4 chain32; do
  run_abc "${fx}_j1"  "read $DIR/$fx.blif; strash; dch2 -j 1; write_blif ${fx}_j1.blif"
  run_abc "${fx}_j4a" "read $DIR/$fx.blif; strash; dch2 -j 4; write_blif ${fx}_j4a.blif"
  run_abc "${fx}_j4b" "read $DIR/$fx.blif; strash; dch2 -j 4; write_blif ${fx}_j4b.blif"
  h1=$(ts_hash ${fx}_j1.blif) h4=$(ts_hash ${fx}_j4a.blif)
  [ "$h1" = "$h4" ] && ok "$fx j1/j4 output identity ($h1)" || bad "$fx j1/j4 output identity ($h1 vs $h4)"
  s1=$(grep -oE "DCH2: .*" ${fx}_j1.log | head -1 | sed 's/verify_ms=[0-9]* merge_ms=[0-9]*//')
  s4=$(grep -oE "DCH2: .*" ${fx}_j4a.log | head -1 | sed 's/verify_ms=[0-9]* merge_ms=[0-9]*//')
  [ "$s1" = "$s4" ] && ok "$fx j1/j4 counter identity ($s1)" || bad "$fx j1/j4 counter identity ($s1 vs $s4)"
  h4b=$(ts_hash ${fx}_j4b.blif)
  [ "$h4" = "$h4b" ] && ok "$fx j4 determinism" || bad "$fx j4 determinism ($h4 vs $h4b)"
done

# ---- 5. counter invariants ---------------------------------------------
echo "== counter invariants"
run_abc inv_v "read $DIR/dup4.blif; strash; dch2 -j 4 -v; write_blif inv_v.blif"
sum=$(grep -m1 -oE "DCH2: .*" inv_v.log)
cand=$(echo "$sum" | sed -E 's/.*DCH2: [0-9]+ windows, ([0-9]+) candidates.*/\1/')
ver=$(echo "$sum" | sed -E 's/.* ([0-9]+) verified,.*/\1/')
rej=$(echo "$sum" | sed -E 's/.* ([0-9]+) rejected,.*/\1/')
und=$(echo "$sum" | sed -E 's/.* ([0-9]+) undecided,.*/\1/')
mer=$(echo "$sum" | sed -E 's/.* ([0-9]+) merged,.*/\1/')
sub=$(echo "$sum" | sed -E 's/.* ([0-9]+) substituted,.*/\1/')
con=$(echo "$sum" | sed -E 's/.* ([0-9]+) conflicts,.*/\1/')
tfi=$(echo "$sum" | sed -E 's/.* ([0-9]+) tfi-skipped,.*/\1/')
drop=$(grep -c "DBG-DROP" inv_v.log || true)
if [ -n "$cand" ] && [ "$cand" -eq $((ver + rej + und)) ]; then
  ok "candidates($cand) == verified($ver)+rejected($rej)+undecided($und)"
else
  bad "candidates($cand) == verified($ver)+rejected($rej)+undecided($und)"
fi
if [ -n "$ver" ] && [ "$ver" -eq $((mer + sub + con + tfi + drop)) ]; then
  ok "verified($ver) == merged($mer)+substituted($sub)+conflicts($con)+tfi($tfi)+dropped($drop)"
else
  bad "verified($ver) == merged($mer)+substituted($sub)+conflicts($con)+tfi($tfi)+dropped($drop)"
fi
# dup4: the three associations form exactly 3 candidate pairs (2 merges,
# 1 same-class conflict after union-find)
[ "$cand" = "3" ] && [ "$ver" = "3" ] && [ "$mer" = "2" ] && [ "$con" = "1" ] \
  && ok "dup4 association matrix (3 candidates, 3 verified, 2 merged, 1 conflict)" \
  || bad "dup4 association matrix (cand=$cand ver=$ver mer=$mer con=$con)"

# ---- 6. CHECK ------------------------------------------------------------
echo "== CHECK"
run_abc check_dup4 "set check; read $DIR/dup4.blif; strash; dch2 -j 4; write_blif check_dup4.blif; print_stats"
[ -s check_dup4.blif ] && ok "CHECK run completes with output" || bad "CHECK run completes with output"
run_abc check_add8 "set check; read $DIR/add8.blif; strash; dch2 -j 1 -v; write_blif check_add8.blif"
[ -s check_add8.blif ] && ok "CHECK run (j1 -v) completes with output" || bad "CHECK run (j1 -v) completes"

# ---- 7. post-DCH and post-mapping CEC -------------------------------------
echo "== CEC"
for fx in add8 dup4 chain32; do
  cec_out=$(run_abc "${fx}_cec" "read $DIR/$fx.blif; strash; cec ${fx}_j4a.blif")
  grep -q "Networks are equivalent" ${fx}_cec.log && ok "$fx post-DCH CEC equivalent" || bad "$fx post-DCH CEC equivalent"
done
printf '1 1.00 1.00\n2 1.00 1.00\n3 1.00 1.00\n4 1.00 1.00\n' > lutdefs4.txt
for fx in add8 dup4 chain32; do
  run_abc "${fx}_map" "read_lut lutdefs4.txt; read ${fx}_j4a.blif; strash; if; mfs2; lutpack -S 1; write_blif ${fx}_mapped.blif"
  run_abc "${fx}_mapcec" "read $DIR/$fx.blif; strash; cec ${fx}_mapped.blif"
  grep -q "Networks are equivalent" ${fx}_mapcec.log && ok "$fx post-mapping CEC equivalent" || bad "$fx post-mapping CEC equivalent"
done

# ---- 8. timeout/fault sensitivity ------------------------------------------
echo "== timeout sensitivity"
run_abc toc1  "read $DIR/add8.blif; strash; dch2 -j 4 -C 1; write_blif toc1.blif"
run_abc toc1cec  "read $DIR/add8.blif; strash; cec toc1.blif"
grep -q "Networks are equivalent" toc1cec.log && ok "add8 -C 1 output CEC equivalent" || bad "add8 -C 1 output CEC equivalent"
u1=$(grep -m1 -oE "DCH2: .*" toc1.log | sed -E 's/.* ([0-9]+) undecided,.*/\1/')
[ -n "$u1" ] && [ "$u1" -ge 1 ] && ok "add8 -C 1 produces undecided outcomes ($u1)" || bad "add8 -C 1 produces undecided outcomes ($u1)"
run_abc toc100 "read $DIR/chain32.blif; strash; dch2 -j 4 -C 100; write_blif toc100.blif"
run_abc toc100cec "read $DIR/chain32.blif; strash; cec toc100.blif"
grep -q "Networks are equivalent" toc100cec.log && ok "chain32 -C 100 rejection path CEC equivalent" || bad "chain32 -C 100 rejection path CEC equivalent"

# ---- 9. fault injection (test hook) -----------------------------------------
echo "== fault injection"
hbase=$(ts_hash dup4_j4a.blif)
vbase=$(grep -m1 -oE "DCH2: .*" dup4_j4a.log | sed -E 's/.*DCH2: [0-9]+ windows, ([0-9]+) candidates, ([0-9]+) verified.*/\2/')
for PAIR in "0,1" "1,2"; do
  itag="inj_$(echo $PAIR | tr ',' '_')"
  DCH2_TEST_INJECT=$PAIR "$ABC" -f /dev/stdin > $itag.log 2>&1 <<EOF
read $DIR/dup4.blif
strash
dch2 -j 4 -v
write_blif $itag.blif
EOF
  hinj=$(ts_hash $itag.blif)
  [ "$hinj" = "$hbase" ] && ok "inject $PAIR: false pair rejected, output identical" || bad "inject $PAIR: false pair rejected (hash moved)"
  vinj=$(grep -m1 -oE "DCH2: .*" $itag.log | sed -E 's/.*DCH2: [0-9]+ windows, ([0-9]+) candidates, ([0-9]+) verified.*/\2/')
  [ "$vinj" = "$vbase" ] && ok "inject $PAIR: verified count unchanged ($vinj)" || bad "inject $PAIR: verified count unchanged ($vbase vs $vinj)"
done

# ---- 10. default-off absence of test hooks ----------------------------------
echo "== default-off"
run_abc defoff "read $DIR/dup4.blif; strash; dch2 -j 4 -v; write_blif defoff.blif"
grep -q "DCH2-TEST-INJECT" defoff.log && bad "no injection markers when env absent" || ok "no injection markers when env absent"
hdef=$(ts_hash defoff.blif)
[ "$hdef" = "$hbase" ] && ok "default-off output identical to baseline" || bad "default-off output identical to baseline"

echo "DCH2-TEST done FAIL=$FAIL"
exit $FAIL
