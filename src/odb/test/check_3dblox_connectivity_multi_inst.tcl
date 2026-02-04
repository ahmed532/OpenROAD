# check_3dblox_connectivity_multi_inst.tcl
source "helpers.tcl"

read_3dbx data/top_multi.3dbx

check_3dblox -bump_pitch_tolerance 5

set errors [get_3dblox_marker_count "3DBlox"]

if { $errors != 0 } {
  puts "FAIL: Found $errors errors"
  exit 1
} else {
  puts "PASS: Found 0 errors"
  exit 0
}
