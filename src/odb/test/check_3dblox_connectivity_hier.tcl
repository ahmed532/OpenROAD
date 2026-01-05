source "helpers.tcl"

read_3dbx "data/recursive/top.3dbx"

# 1. Initial State: Should match (1000 vs 1000)
check_3dblox
set err1 [get_3dblox_connected_errors]
check "Initial Connection Match" { set err1 } 0

# 2. Move mid_inst1 to break connection
set inst [[[ord::get_db] getChip] findChipInst "mid_inst1"]
lassign [$inst getLoc] x y z
set master [$inst getMasterChip]
set w [$master getWidth]
set h [$master getHeight]

set p [odb::Point3D]
$p set [expr $x + 2 * $w] [expr $y + 2 * $h] $z
$inst setLoc $p

check_3dblox
set err2 [get_3dblox_connected_errors]

# Note: getMarkerCount accumulates?
# check_3dblox calls createOrReplace.
# So count should be from latest run?
# checkConnectionRegions calls createOrReplace(category, "Connected regions").
# So it resets the subcategory markers. Yes.
check "Moved Connection Mismatch" { set err2 } 1

exit_summary
