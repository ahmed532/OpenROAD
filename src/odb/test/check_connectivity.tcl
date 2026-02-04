source "helpers.tcl"

# Read the 3dbx file (which automatically calls check3DBlox)
if { [catch { read_3dbx "data/test_connectivity.3dbx" } err_msg] } {
  puts "Read 3dbx reported error (as expected due to markers): $err_msg"
}

# Verification
set db [ord::get_db]
set chip [$db getChip]
set inst [$chip findChipInst "soc_inst"]
set master [$inst getMasterChip]

set nets [$master getChipNets]
set clk_found 0
set rst_found 0
set data_found 0

# Check nets
foreach net $nets {
  set name [$net getName]
  if { $name == "clk" } {
    set clk_found 1
  }
  if { $name == "rst" } {
    set rst_found 1
  }
  if { $name == "data_bus[0]" } {
    set data_found 1
  }
}

if { $clk_found == 0 } {
  puts "ERROR: Net 'clk' not found on master chip"
  exit 1
} else {
  puts "SUCCESS: Net 'clk' found"
}

if { $rst_found == 0 } {
  puts "ERROR: Net 'rst' not found on master chip"
  exit 1
} else {
  puts "SUCCESS: Net 'rst' found"
}

if { $data_found == 0 } {
  puts "ERROR: Net 'data_bus[0]' not found on master chip"
  exit 1
} else {
  puts "SUCCESS: Net 'data_bus[0]' found"
}

puts "Marker Categories on chip [[$db getChip] getName]:"
set top_cats [[$db getChip] getMarkerCategories]
foreach c $top_cats {
  puts "Category: [$c getName]"
  set sub_cats [$c getMarkerCategories]
  foreach sc $sub_cats {
    puts "  Sub-category: [$sc getName], Markers: [llength [$sc getMarkers]]"
  }
}

set cat [$chip findMarkerCategory "3DBlox"]
if { $cat == "NULL" } {
  puts "ERROR: No 3DBlox marker category"
  exit 1
}
set align_cat [$cat findMarkerCategory "Logical Alignment"]
if { $align_cat == "NULL" } {
  puts "ERROR: No Logical Alignment marker category"
  exit 1
}
set markers [$align_cat getMarkers]
if { [llength $markers] != 1 } {
  puts "ERROR: Expected 1 marker for bad_net, found [llength $markers]"
  foreach m $markers {
    puts "Marker: [$m getComment]"
  }
  exit 1
} else {
  puts "SUCCESS: Found 1 marker as expected: [[lindex $markers 0] getComment]"
}

set design_cat [$cat findMarkerCategory "Design Alignment"]
if { $design_cat == "NULL" } {
  puts "SUCCESS: No Design Alignment markers found (Rule 1 passed)"
} else {
  set design_markers [$design_cat getMarkers]
  if { [llength $design_markers] != 0 } {
    puts "ERROR: Found [llength $design_markers] Design Alignment markers (Rule 1 failed?)"
    exit 1
  } else {
    puts "SUCCESS: Design Alignment category exists but is empty"
  }
}

exit_summary
