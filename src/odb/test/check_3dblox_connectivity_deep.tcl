source "helpers.tcl"

# Load design with 3-level hierarchy
read_3dbx "data/recursive/deep_top.3dbx"
check_3dblox

# Expect 0 errors
# Path: upper(1000) + mid(1000) + leaf(-1000) = 1000
# Bot: leaf_inst_top(1000) = 1000
# Should match
set conn_errors [get_3dblox_connected_errors]

check "Deep recursion alignment" { set conn_errors } 0

exit_summary
