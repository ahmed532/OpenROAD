source "helpers.tcl"

# 1. Load design with hierarchical connection
read_3dbx "data/recursive/top.3dbx"
set db [ord::get_db]
set top_chip [$db getChip]

# Verify initial load
puts "Loaded design: [$top_chip getName]"

# 2. Check connections (Should be connected)
check_3dblox

# With CORRECT implementation: 0 errors (connected)
# With ORIGINAL (buggy) implementation: 1 error (false positive disconnect)
set conn_errors [get_3dblox_marker_count "Connection"]

check "Recursive connection check" { set conn_errors } 0

exit_summary
