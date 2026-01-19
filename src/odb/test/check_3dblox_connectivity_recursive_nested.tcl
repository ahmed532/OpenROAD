source helpers.tcl

read_3dbx data/recursive/hard_nested_top.3dbx
check_3dblox
set errors [get_3dblox_marker_count "Connected regions"]

check "Hard Nested connection match" { set errors } 0

# Move mid_inst to break connection
set db [ord::get_db]
set top_chip [$db getChip]
set mid [$top_chip findChipInst "mid_inst"]
lassign [$mid getLoc] mx my mz
set mmaster [$mid getMasterChip]
set mw [$mmaster getWidth]
set mh [$mmaster getHeight]

set p [odb::Point3D]
$p set [expr $mx + 2 * $mw] [expr $my + 2 * $mh] $mz
$mid setLoc $p
check_3dblox
set move_errors [get_3dblox_marker_count "Connected regions"]
puts "Hard Nested move errors: $move_errors"
check "Hard Nested move check" { expr $move_errors > 0 } 1

exit_summary
