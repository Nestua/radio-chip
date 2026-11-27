#!/bin/bash
# Runs of a design through the open ASIC flow on the build server, several at a time, with an
# early reading of each (docs/gates/silicon_probe.md). Where tools/syn/orfs_block.sh runs one
# block and waits, this starts a run from another run's directory (its config.mk, constraint,
# macros), changes a few settings, and leaves it running. Evidence for the gate journals, not
# a tape-out flow.
#
#   probe_run.sh start  <name> <template> <src> [VAR=value | -VAR] ...
#   probe_run.sh status [name ...]        every run of the root if none is named
#   probe_run.sh wait   <name> ...        returns when all have ended
#   probe_run.sh keep   <name>            reports, logs, final database → asic-<name>-keep
#   probe_run.sh paths  <name> [stage]    failing endpoints in groups, the worst paths
#                                         (stage: place, cts, grt, final or a database's name)
#   probe_run.sh stop   <name>
#
# A run lives in $PROBE_ROOT/asic-<name> (default root /data/radio-chip). <template> is a run
# to copy the set-up from (its files outside run/, less its logs and end mark); <src> is a tree with hw/rtl, mounted
# read-only as /repo. VAR=value sets or replaces a line of config.mk, -VAR removes one;
# PROBE_CORES (default 8) is the threads a run may use, so that four fit the server.
#
# Reading a run early: `status` prints the worst setup slack after placement, the clock tree
# and global routing as each report appears. Those are estimates before routing: over runs A
# to G of 2026-11-27 the one after global routing was 0.4 to 0.9 ns below the final slack,
# and they kept the runs' order but for one. So they rank variants (stop the clearly worse
# ones) and decide nothing.
#
# Needs docker, the image openroad/orfs (PROBE_IMAGE), GNU sed and cpio on the machine it runs
# on. `paths` reads the IHP SG13G2 standard-cell library only; before the final stage it
# estimates wires from the placement, also for the stage after global routing.
set -euo pipefail

root=${PROBE_ROOT:-/data/radio-chip}
image=${PROBE_IMAGE:-openroad/orfs:latest}
cores=${PROBE_CORES:-8}
orfs='source /OpenROAD-flow-scripts/env.sh >/dev/null 2>&1'

die() { echo "probe_run: $*" >&2; exit 1; }
run_dir() { echo "$root/asic-$1"; }

# The design's directory inside a run (the one holding config.mk), its name and platform.
design_dir() {
    local d
    d=$(dirname "$(find "$(run_dir "$1")" -mindepth 2 -maxdepth 2 -name config.mk | head -1)")
    [ -f "$d/config.mk" ] || die "no config.mk in $(run_dir "$1")"
    echo "$d"
}
cfg() { sed -n "s/^export $2 *?*= *//p" "$(design_dir "$1")/config.mk" | head -1; }
base() {  # reports | logs | results of a run
    echo "$(design_dir "$1")/run/$2/$(cfg "$1" PLATFORM)/$(cfg "$1" DESIGN_NAME)/base"
}
slack_of() {  # the worst setup slack a report states last, or "-"
    [ -f "$1" ] && grep -A2 '^worst slack max' "$1" | grep -Eo -- '-?[0-9]+\.[0-9]+' | tail -1 || echo "-"
}

cmd_start() {
    [ $# -ge 3 ] || die "start <name> <template> <src> [VAR=value | -VAR] ..."
    local name=$1 tmpl=$2 src; src=$(realpath "$3"); shift 3
    local d; d=$(run_dir "$name")
    [ -e "$d" ] && die "$d exists (stop and remove it, or take another name)"
    [ -d "$root/asic-$tmpl" ] || die "no template run $root/asic-$tmpl"
    [ -d "$src/hw/rtl" ] || die "no hw/rtl under $src"
    case $name in *-keep|*[!A-Za-z0-9_.-]*) die "name '$name': letters, digits, _ . - only, not ending in -keep" ;; esac
    for s in "$@"; do
        case $s in -?*|?*=*) ;; *) die "setting '$s' is neither VAR=value nor -VAR" ;; esac
    done
    find "$root/asic-$tmpl" -mindepth 2 -maxdepth 2 -name config.mk | grep -q . \
        || die "no <design>/config.mk in the template $root/asic-$tmpl"
    mkdir -p "$d"
    # the set-up only: no results, logs or end marks of the template
    (cd "$root/asic-$tmpl" && find . -path '*/run' -prune -o -type f \
        ! -name flow.log ! -name DONE ! -name run.out ! -name 'paths_*' -print0 | cpio -0pdm --quiet "$d")
    local dd; dd=$(design_dir "$name")
    local top; top=$(basename "$dd")
    for s in "$@"; do
        local var=${s%%=*}; var=${var#-}
        sed -i "/^export $var *?*=/d" "$dd/config.mk"
        case $s in
            -*) ;;
            *=*) echo "export $var = ${s#*=}" >> "$dd/config.mk" ;;
            *) die "setting '$s' is neither VAR=value nor -VAR" ;;
        esac
    done
    echo "$src" > "$d/SOURCE"
    docker run -d --rm --name "probe-$name" -v "$src":/repo:ro -v "$d":/work "$image" bash -lc \
        "$orfs; cd /OpenROAD-flow-scripts/flow; \
         make DESIGN_CONFIG=/work/$top/config.mk WORK_HOME=/work/$top/run NUM_CORES=$cores \
           > /work/$top/flow.log 2>&1; echo \$? > /work/DONE" > /dev/null
    echo "started $name: $d ($cores threads, source $src)"
}

cmd_status() {
    local names=("$@")
    if [ ${#names[@]} -eq 0 ]; then
        for d in "$root"/asic-*; do
            case $d in *-keep) continue ;; esac
            [ -d "$d" ] && find "$d" -mindepth 2 -maxdepth 2 -name config.mk | grep -q . \
                && names+=("$(basename "$d" | sed 's/^asic-//')")
        done
    fi
    printf '%-10s %-22s %7s %7s %7s %7s\n' run state place cts grt final
    for n in "${names[@]}"; do
        local r l state
        r=$(base "$n" reports); l=$(base "$n" logs)
        if [ -f "$(run_dir "$n")/DONE" ]; then
            state="ended ($(cat "$(run_dir "$n")/DONE"))"
            # runs started by hand wrote the word "done" there
        else
            state=$({ ls -t "$l" 2>/dev/null || true; } | head -1 | sed 's/\.tmp\.log$//; s/\.log$//; s/\.json$//')
            state="at ${state:-start}"
            docker ps --format '{{.Names}}' | grep -qx -e "probe-$n" -e "$n" || state="died $state"
        fi
        printf '%-10s %-22s %7s %7s %7s %7s\n' "$n" "$state" \
            "$(slack_of "$r/3_detailed_place.rpt")" "$(slack_of "$r/4_cts_final.rpt")" \
            "$(slack_of "$r/5_global_route.rpt")" "$(slack_of "$r/6_finish.rpt")"
    done
}

cmd_wait() {
    [ $# -ge 1 ] || die "wait <name> ..."
    for n in "$@"; do
        [ -d "$(run_dir "$n")" ] || die "no run $(run_dir "$n")"
        until [ -f "$(run_dir "$n")/DONE" ]; do
            docker ps --format '{{.Names}}' | grep -qx -e "probe-$n" -e "$n" \
                || { echo "probe_run: $n has no container and no end mark (died)" >&2; break; }
            sleep 30
        done
    done
    cmd_status "$@"
}

# Static timing on a stage's database of a run: the failing endpoints and the worst paths.
# The final stage is read with its extracted parasitics; an earlier one with the flow's
# estimate of the wires.
cmd_paths() {
    [ $# -ge 1 ] || die "paths <name> [stage, default 6_final]"
    local n=$1 stage=${2:-6_final} d dd top res out
    case $stage in
        place) stage=3_place ;; cts) stage=4_cts ;; grt) stage=5_1_grt ;; final) stage=6_final ;;
    esac
    d=$(run_dir "$n"); dd=$(design_dir "$n"); top=$(basename "$dd")
    res=$(base "$n" results); out="$d/paths_$stage"
    [ -f "$res/$stage.odb" ] || die "no $res/$stage.odb"
    local inres=${res#"$d"}
    {
        echo "read_liberty /OpenROAD-flow-scripts/flow/platforms/$(cfg "$n" PLATFORM)/lib/sg13g2_stdcell_typ_1p20V_25C.lib"
        for lib in $(cfg "$n" ADDITIONAL_TYP_LIBS); do echo "read_liberty $lib"; done
        echo "read_db /work$inres/$stage.odb"
        if [ -f "$res/$stage.sdc" ]; then echo "read_sdc /work$inres/$stage.sdc"
        else echo "read_sdc /work/$top/constraint.sdc"; fi
        echo "source /OpenROAD-flow-scripts/flow/platforms/$(cfg "$n" PLATFORM)/setRC.tcl"
        if [ -f "$res/$stage.spef" ]; then echo "read_spef /work$inres/$stage.spef"
        else echo "estimate_parasitics -placement"; fi
        echo 'puts "@@ends"'
        echo "report_checks -path_delay max -group_path_count 2000 -endpoint_path_count 1 -format end -slack_max 0 -digits 3"
        echo 'puts "@@paths"'
        echo "report_checks -path_delay max -group_path_count 40 -endpoint_path_count 1 -format full -digits 2"
        echo "exit"
    } > "$out.tcl"
    docker run --rm -v "$d":/work "$image" bash -lc \
        "$orfs; openroad -no_init -exit /work/$(basename "$out.tcl")" > "$out.out" 2>&1 \
        || die "static timing failed, see $out.out"
    echo "== $n, $stage: failing endpoints by group (count, worst slack, sum)"
    # the end-format list only (between the two marks); a long name is alone on the line before
    awk '/^@@paths/ { exit } /^@@ends/ { p = 1 }
         p && /VIOLATED/ { print (NF >= 5 ? $1 : prev), $(NF-1) } { prev = $1 }' "$out.out" > "$out.ends"
    sed -E 's/(\$_[A-Za-z0-9_]+_)?(\/[A-Za-z_]+)?(\[[0-9]+\])? / /; s/\[[0-9]+\]//g' "$out.ends" \
        | awk '{n[$1]++; s[$1]+=$2; if(!($1 in w)||$2<w[$1])w[$1]=$2}
               END{for(k in n)printf "%4d %7.2f %8.2f  %s\n", n[k], w[k], s[k], k}' | sort -k2n
    echo "   total: $(wc -l < "$out.ends" | tr -d ' ') failing endpoints (the list is cut at 2000 a path group)"
    echo "== the worst paths (slack, pins after the launch's clock pin, their delay, start -> end)"
    awk '/^@@paths/{p=1} p' "$out.out" | awk '
        /^Startpoint:/ {sp=$2; n=0; t0=""}
        /^Endpoint:/   {ep=$2}
        /^ +[0-9.-]+ +[0-9.]+ [v^] / { if ($4 ~ /\//) { if (t0=="") t0=$2; else n++; t1=$2 } }
        /slack \(/     {printf "%6s %3d pins %5.2f ns  %s -> %s\n", $1, n, t1-t0, sp, ep}' \
        | sed -E 's/\[[0-9]+\]//g; s/\$_[A-Z_0-9]+_//g' | sort -n | awk '!seen[$6 $8]++ && ++c <= 12'
    echo "   (the count is of output pins on the path, the launch's and its clock tree's included,"
    echo "    and the endpoint's input; full report: $out.out)"
}

cmd_keep() {
    [ $# -eq 1 ] || die "keep <name>"
    local n=$1 d dd res rep k
    d=$(run_dir "$n"); dd=$(design_dir "$n"); res=$(base "$n" results); rep=$(base "$n" reports)
    [ -f "$res/6_final.odb" ] || die "$n has no final database (not ended, or failed)"
    k="$root/asic-$n-keep"
    [ -e "$k" ] && die "$k exists (move it away first)"
    mkdir -p "$k/results"
    cp -r "$dd"/config.mk "$dd"/constraint.sdc "$dd/run/reports" "$dd/run/logs" "$k/"
    [ -f "$dd/macros.tcl" ] && cp "$dd/macros.tcl" "$k/"
    [ -f "$d/SOURCE" ] && cp "$d/SOURCE" "$k/"
    cp "$res"/6_final.odb "$res"/6_final.sdc "$res"/6_final.spef "$res"/1_2_yosys.v "$k/results/"
    cmd_paths "$n" 6_final | tee "$k/paths_summary.txt"
    cp "$d/paths_6_final.out" "$k/failing_endpoints.txt"
    {
        echo "== $n: totals"
        grep 'Design area' "$(base "$n" logs)/6_report.log" | tail -1 || true
        echo "worst slack max $(slack_of "$rep/6_finish.rpt")"
        echo "tns max $(grep -A2 '^tns max' "$rep/6_finish.rpt" | grep -Eo -- '-?[0-9]+\.[0-9]+' | tail -1 || true)"
        { grep -h -A3 -E '(setup|hold|max_fanout|max_cap|max_slew)_violation_count' "$rep/6_finish.rpt" || true; } \
            | { grep -E 'count [0-9]+' || true; } | tr '\n' ';'; echo
        echo "power (W: internal switching leakage total): $({ grep -A14 'report_power' "$rep/6_finish.rpt" || true; } \
            | awk '/^Total/{print $2, $3, $4, $5; exit}')"
        echo "synthesised area: $({ grep 'Chip area for' "$rep/synth_stat.txt" || true; } | tail -1 | awk '{print $NF}') um^2"
    } | tee -a "$k/paths_summary.txt"
    echo "kept in $k"
}

cmd_stop() {
    [ $# -eq 1 ] || die "stop <name>"
    [ -d "$(run_dir "$1")" ] || die "no run $(run_dir "$1")"
    [ -f "$(run_dir "$1")/DONE" ] && die "$1 has ended already"
    # a run started by hand has the bare name; only a run with a directory and no end mark gets here
    docker stop "probe-$1" > /dev/null 2>&1 || docker stop "$1" > /dev/null 2>&1 \
        || die "no running container probe-$1"
    echo stopped > "$(run_dir "$1")/DONE"
    echo "stopped $1 (its directory is left as it is)"
}

[ $# -ge 1 ] || die "start | status | wait | keep | paths | stop (see the header)"
c=$1; shift
case $c in
    start|status|wait|keep|paths|stop) "cmd_$c" "$@" ;;
    *) die "unknown command '$c'" ;;
esac
