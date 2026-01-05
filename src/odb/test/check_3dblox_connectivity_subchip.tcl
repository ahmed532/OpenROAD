source "helpers.tcl"

# 1. Load design with connection in sub-chip
# The connection is defined INSIDE sub_mid.3dbx.
# Non-recursive check would ignore it.
read_3dbx "data/recursive/sub_top.3dbx"
check_3dblox

# Expect 0 errors (all connected in sub_mid)
set conn_errors [get_3dblox_connected_errors]
puts "Sub-chip connection errors: $conn_errors"

check "Sub-chip connection detection" { set conn_errors } 0

exit_summary
