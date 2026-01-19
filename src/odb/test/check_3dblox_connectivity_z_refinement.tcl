source "helpers.tcl"

# Set up database
set db [ord::get_db]

# read_3dbx automatically calls check_3dblox
read_3dbx "z_refinement_test/design.3dbx"

# Report markers
set chip [$db getChip]
set categories [$chip getMarkerCategories]
set total_markers 0
foreach category $categories {
  set markers [$category getMarkers]
  set total_markers [expr $total_markers + [llength $markers]]
  foreach marker $markers {
    puts "Marker: [$marker getComment]"
  }
}

if { $total_markers > 0 } {
  puts "FAIL: Got $total_markers markers, expected 0."
  exit 1
}

puts "PASS: 3DBlox Connectivity Check Passed (No false positives due to Z-hack)"
exit 0
