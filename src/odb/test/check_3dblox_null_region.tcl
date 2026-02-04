source "helpers.tcl"

# Create a design with a connection to a non-existent region
# This should trigger the "missing regions" warning

# We can't easily create a "bad" 3dbx that parses but has null regions
# without the parser complaining, so we'll use a valid one and then
# manunally clear the region if the API allows it, or just
# provide a design where the region is not found.

read_3dbx "data/broken_conn.3dbx"

check_3dblox
# Should see ODB-0207 warning (captured by logger usually)
# But here we check if it's skipped correctly and marker isn't created for it

set markers [get_3dblox_marker_count "Connected regions"]
check "Broken connection skipped" { set markers } 0

exit_summary
