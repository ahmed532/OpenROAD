source helpers.tcl

read_3dbx data/recursive/hard_z_top.3dbx
check_3dblox
set errors [get_3dblox_marker_count "Connected regions"]

check "Hard Z connection match" { set errors } 0

exit_summary
