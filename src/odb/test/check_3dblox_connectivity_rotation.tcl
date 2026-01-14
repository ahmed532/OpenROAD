source "helpers.tcl"

# Load design with rotated instance
read_3dbx "data/recursive/rotation_top.3dbx"
check_3dblox

# Expect 0 errors - rotation should be applied correctly
set conn_errors [get_3dblox_connected_errors]

check "Rotation alignment" { set conn_errors } 0

exit_summary
