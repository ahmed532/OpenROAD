source helpers.tcl

read_3dbx data/recursive/hard_mirror_top.3dbx
check_3dblox
set errors [get_3dblox_marker_count "Connected regions"]

check "Hard Mirror connection match" { set errors } 0

exit_summary
