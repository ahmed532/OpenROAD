source "helpers.tcl"

read_3dbx "data/hard_2.3dbx"
set top_chip [[ord::get_db] getChip]

check_3dblox
check "Hard 2: Hierarchical Rotation (valid)" { get_3dblox_marker_count "Connected regions" } 0

# Move targets
# Move targets far away
set target_inst [$top_chip findChipInst "target_inst"]
lassign [$target_inst getLoc] tx ty tz
set tmaster [$target_inst getMasterChip]
set tw [$tmaster getWidth]
set th [$tmaster getHeight]

set p [odb::Point3D]
$p set [expr $tx + 10 * $tw] [expr $ty + 10 * $th] $tz
$target_inst setLoc $p

check_3dblox
check "Hard 2: Hierarchical Rotation (broken)" { get_3dblox_marker_count "Connected regions" } 1

# puts "pass"
exit_summary
