source "helpers.tcl"

# Load design with Z mismatch
read_3dbx "data/recursive/z_top.3dbx"
check_3dblox

# Expect 1 mismatch error because Z is different
set conn_errors [get_3dblox_marker_count "Connected regions"]

check "Z-axis mismatch detection" { set conn_errors } 1

exit_summary
