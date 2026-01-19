source "helpers.tcl"

# Setup database and tech
set db [ord::get_db]
set tech [odb::dbTech_create $db "tech"]
set top_chip [odb::dbChip_create $db $tech "TopChip" "HIER"]
$top_chip setWidth 10000
$top_chip setHeight 10000
$top_chip setThickness 1000

# Create master chips
set chip1 [odb::dbChip_create $db $tech "Chip1" "DIE"]
$chip1 setWidth 2000
$chip1 setHeight 2000
$chip1 setThickness 500

set chip2 [odb::dbChip_create $db $tech "Chip2" "DIE"]
$chip2 setWidth 1500
$chip2 setHeight 1500
$chip2 setThickness 500

# Create overlapping chip instances
set inst1 [odb::dbChipInst_create $top_chip $chip1 "inst1"]
set p1 [odb::Point3D]
$p1 set 0 0 0
$inst1 setLoc $p1

set inst2 [odb::dbChipInst_create $top_chip $chip2 "inst2"]
# Overlap inst2 with inst1
set p2 [odb::Point3D]
$p2 set 1000 1000 0
$inst2 setLoc $p2

# Run checker
check_3dblox

# Verify markers
set category [$top_chip findMarkerCategory "3DBlox"]
if { $category == "NULL" } {
  puts "FAIL: 3DBlox category not found"
  exit 1
}

set overlapping_category [$category findMarkerCategory "Overlapping chips"]
if { $overlapping_category == "NULL" } {
  puts "FAIL: Overlapping chips category not found"
  exit 1
}

set marker_count [$overlapping_category getMarkerCount]
if { $marker_count == 0 } {
  puts "FAIL: No overlap markers found"
  exit 1
}

puts "pass"
exit
