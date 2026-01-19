# This test verifies the refinement of 3DBlox connectivity checking, specifically 
# handling cases where a simple cuboid-based intersection check (bounding box) 
# would lead to false positive connectivity markers in the Z-dimension.
#
# The test uses a design where objects overlap in X-Y but are separated by 
# layers in Z such that they should NOT be considered connected.
#
# Expectations:
# - PASS: With the refined layer-based connectivity check, no markers 
#   are generated (expected 0).
# - FAIL: The simple cuboid approach would detect an intersection in Z and 
#   generate false positive "Connected regions" markers.

source "helpers.tcl"

# Set up database
set db [ord::get_db]

# read_3dbx automatically calls check_3dblox
read_3dbx "z_refinement_test/design.3dbx"

set total_markers [get_3dblox_marker_count "Connected regions"]

if { $total_markers > 0 } {
  puts "FAIL: Got $total_markers markers, expected 0."
  exit 1
}

puts "PASS: 3DBlox Connectivity Check Passed (No false positives due to Z-hack)"
exit 0