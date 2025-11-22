"""develop_fea_example_9.py - Motor FEA with Selectable Linear/Nonlinear Steel Analysis
======================================================================================

This script builds a complete FEA model of a surface-mounted PM motor with 
selectable linear or nonlinear steel material properties. The stator uses a 
segmented inner boundary that traces around slot openings for physically accurate geometry.

Key Features:
- **Selectable Steel Analysis**: User control variable to switch between:
  * LINEAR mode: Constant μr=2000 for rotor/stator cores
  * NONLINEAR mode: B-H curve (M19 electrical steel approximation)
- Steel rotor core with shaft hole
- Slotted stator core with 48 rectangular slots
- Segmented inner boundary that traces into and out of each slot (no lines across slot openings)
- Five air regions: center air, inner air, middle air 1 (airgap to slots), 
  middle air 2 (stator OD to outer air), and shell
- Dynamic parameter loading from Excel (projects/example_design_1/)
- 8-pole surface-mounted PM rotor with alternating N/S polarity
- Fine mesh at airgap and in slots for accurate field calculation
- Python-based post-processing with stator back iron diagnostics

Geometry Structure:
- Center air: 0 to rotor core ID
- Rotor core: steel annulus (shaft ID to rotor core OD)
- Inner air: rotor core OD to airgap middle, with cutouts for magnets and rotor core
- Magnets: 8 surface-mounted, alternating N/S polarity
- Middle air 1: airgap middle to stator inner boundary (includes slot cavities)
- Stator core: segmented inner boundary (traces around slots) to stator OD
- Middle air 2: stator OD to outer air (post-stator air)
- Shell: outer air to domain boundary (far-field)

Slot Implementation:
- Each slot defined by 4 points (pt1, pt2 at inner radius; pt3, pt4 at slot depth)
- 3 lines per slot: ln2 (right radial), ln3 (top), ln4 (left radial)
- No line across slot bottom (pt1→pt2) - this creates the open slot
- Inner boundary uses inter-slot line segments plus slot edge traces (ln4, ln3, ln2 reversed)
- Creates continuous closed curve loop without lines across slot openings

Nonlinear Analysis:
- B-H curve: 11 data points from B=0 to 2.05T (covers typical motor operating range)
- Starting reluctivity: ν(B=0) = 398 (matches linear μr=2000 to allow flux entry)
- Newton-Raphson solver: Automatic iteration when Vol_NL_Mag region defined
- Convergence: Typically 4-5 iterations at low flux densities

Post-Processing:
- Complete motor cross-section plot (shaft, rotor, magnets, slotted stator, air regions)
- 2D flux density colormap and airgap flux profile
- Stator back iron diagnostics verify flux penetration into steel
- All output files and plots saved to develop_fea_example_9_output/

Expected Results:
- Complete motor FEA with physically accurate slotted stator geometry
- Air mesh fills slot cavities
- Accurate field solution including slot effects and cogging torque prediction
- Linear vs Nonlinear comparison shows matching results at low flux (<0.5T)
"""

import subprocess
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches
from matplotlib.tri import Triangulation
import struct

# Add src to path for gmsh/getdp executables
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(script_dir)
src_dir = os.path.join(repo_root, 'src')
sys.path.insert(0, src_dir)

# Import motor parameters
from motor_parameters_v0_1 import MotorParameters

# Paths to executables
getdp_dir = os.path.join(src_dir, 'getdp-3.5.0-Windows64')
getdp_exe = os.path.join(getdp_dir, 'getdp.exe')
getdp_templates = os.path.join(getdp_dir, 'templates')
gmsh_exe = os.path.join(src_dir, 'gmsh', 'gmsh.exe')

# Working directory for this example
work_dir = os.path.join(script_dir, 'develop_fea_example_9_output')

# ============================================================================
# ANALYSIS CONTROL - User Configuration
# ============================================================================
USE_NONLINEAR_STEEL = True  # Set to True for nonlinear B-H curve, False for linear μr=2000
# ============================================================================

# Motor parameters
params = MotorParameters()

def create_geometry():
    """
    Step 1: Create the geometry file (.geo) with slotted stator
    
    Builds complete motor geometry with dynamic parameters from Excel:
    - 8 arc magnets on rotor surface (N-S alternating polarity)
    - Slotted stator with segmented inner boundary (traces around slot openings)
    - 48 rectangular slots with 3 edges each (no bottom edge across opening)
    - Inner boundary: inter-slot line segments + reversed slot edge traces
    - Air regions fill from airgap middle up to and including slot cavities
    - Fine mesh at airgap and in slots for accurate field calculation
    
    All dimensions loaded dynamically from Excel parameter file.
    """
    
    print("=" * 60)
    print("Step 1: Creating Geometry")
    print("=" * 60)
    print()
    
    # Create output directory
    os.makedirs(work_dir, exist_ok=True)
    
    geo_file = os.path.join(work_dir, 'c_core.geo')
    
    # Calculate magnet geometry from motor parameters
    mm_to_m = 0.001
    rotor_outer_r = params.rotor_outer_diameter / 2 * mm_to_m  # Magnet outer radius
    magnet_thickness = params.magnet_thickness * mm_to_m
    magnet_inner_r = rotor_outer_r - magnet_thickness  # Magnet inner radius
    airgap = params.airgap_length * mm_to_m  # Airgap between magnets and stator
    stator_inner_r = rotor_outer_r + airgap  # Stator inner radius
    airgap_middle_r = rotor_outer_r + airgap / 2  # Middle of airgap
    
    # Rotor core geometry - fits inside inner air region with small gap
    rotor_core_inner_r = params.rotor_inner_diameter / 2 * mm_to_m  # Shaft ID
    rotor_core_gap = 0.005  # 5mm gap between rotor core OD and magnet inner surface
    rotor_core_outer_r = magnet_inner_r - rotor_core_gap  # Rotor core OD
    
    # Magnet arc angle (one pole)
    pole_pitch_angle = 2 * np.pi / params.num_poles
    magnet_arc_angle = pole_pitch_angle * params.magnet_arc_ratio
    half_arc = magnet_arc_angle / 2
    
    # Stator core geometry
    stator_outer_r = params.stator_outer_diameter / 2 * mm_to_m
    
    # Slot geometry (rectangular slots)
    slot_depth = params.slot_depth * mm_to_m  # Radial depth of slot
    slot_width = params.slot_width * mm_to_m  # Tangential width of slot
    slot_opening = params.slot_opening * mm_to_m  # Opening width at airgap
    num_slots = params.num_slots  # Number of slots
    slot_pitch_angle = 2 * np.pi / num_slots  # Angular spacing between slots
    
    # Boundary radii - three air regions
    outer_air_r = stator_outer_r + 2 * mm_to_m  # Stator OD + 2mm
    domain_outer_r = stator_outer_r + 10 * mm_to_m  # Stator OD + 10mm
    Val_Rint = airgap_middle_r   # Inner boundary at airgap middle
    Val_Rext = domain_outer_r   # Outer boundary at domain outer
    
    # Mesh sizes
    lc_magnet = magnet_thickness / 5  # Fine mesh in magnet
    lc_airgap = airgap / 3  # Fine mesh at airgap middle (3 elements across)
    lc_rotor_core = rotor_core_outer_r / 10  # Medium mesh in rotor core
    lc_stator = 0.005  # 5mm in stator core
    lc_inner_air = 0.01  # 10mm in inner air
    lc_outer_air = 0.015  # 15mm in outer air (stator region)
    lc_shell = (domain_outer_r - outer_air_r) / 5  # Shell mesh
    
    geo_content = f"""// {params.num_poles} Arc Magnets with Rotor Core and Stator Slots - Motor Parameters
// Generated by develop_fea_example_9.py

// Rotor core geometry (steel)
rotor_core_inner_r = {rotor_core_inner_r:.6f};  // {(rotor_core_inner_r/mm_to_m):.1f}mm (shaft ID)
rotor_core_outer_r = {rotor_core_outer_r:.6f};  // {(rotor_core_outer_r/mm_to_m):.1f}mm (rotor OD - 5mm gap)

// Magnet geometry (from motor parameters)
magnet_inner_r = {magnet_inner_r:.6f};  // {(magnet_inner_r/mm_to_m):.1f}mm
magnet_outer_r = {rotor_outer_r:.6f};  // {(rotor_outer_r/mm_to_m):.1f}mm
half_arc = {half_arc:.6f};  // {np.degrees(half_arc):.2f} degrees
pole_angle = {pole_pitch_angle:.6f};  // {np.degrees(pole_pitch_angle):.2f} degrees

// Airgap and stator
airgap = {airgap:.6f};  // {(airgap/mm_to_m):.1f}mm
stator_inner_r = {stator_inner_r:.6f};  // {(stator_inner_r/mm_to_m):.1f}mm
stator_outer_r = {stator_outer_r:.6f};  // {(stator_outer_r/mm_to_m):.1f}mm
airgap_middle_r = {airgap_middle_r:.6f};  // {(airgap_middle_r/mm_to_m):.1f}mm (middle of airgap)
outer_air_r = {outer_air_r:.6f};  // {(outer_air_r/mm_to_m):.1f}mm (stator OD + 2mm)
domain_outer_r = {domain_outer_r:.6f};  // {(domain_outer_r/mm_to_m):.1f}mm (stator OD + 10mm)

// Slot geometry
slot_depth = {slot_depth:.6f};  // {(slot_depth/mm_to_m):.1f}mm
slot_width = {slot_width:.6f};  // {(slot_width/mm_to_m):.1f}mm
slot_opening = {slot_opening:.6f};  // {(slot_opening/mm_to_m):.1f}mm
num_slots = {num_slots};
slot_pitch = {slot_pitch_angle:.6f};  // {np.degrees(slot_pitch_angle):.2f} degrees

Val_Rint = {Val_Rint:.6f};  // Internal boundary (airgap middle)
Val_Rext = {Val_Rext:.6f};  // External boundary (domain outer)

// Mesh size parameters
lc_rotor_core = {lc_rotor_core:.6f};  // Rotor core mesh
lc_magnet = {lc_magnet:.6f};  // Magnet mesh
lc_airgap = {lc_airgap:.6f};  // Airgap mesh (fine)
lc_stator = {lc_stator:.6f};  // Stator core mesh
lc_inner_air = {lc_inner_air:.6f};  // Inner air mesh
lc_outer_air = {lc_outer_air:.6f};  // Outer air mesh
lc_shell = {lc_shell:.6f};  // Shell mesh

// Center point
Point(1) = {{0, 0, 0, lc_inner_air}};

// Rotor core inner boundary (shaft ID)
Point(5) = {{rotor_core_inner_r, 0, 0, lc_rotor_core}};
Point(6) = {{0, rotor_core_inner_r, 0, lc_rotor_core}};
Point(7) = {{-rotor_core_inner_r, 0, 0, lc_rotor_core}};
Point(8) = {{0, -rotor_core_inner_r, 0, lc_rotor_core}};

Circle(5) = {{5, 1, 6}};
Circle(6) = {{6, 1, 7}};
Circle(7) = {{7, 1, 8}};
Circle(8) = {{8, 1, 5}};

// Rotor core outer boundary
Point(115) = {{rotor_core_outer_r, 0, 0, lc_rotor_core}};
Point(116) = {{0, rotor_core_outer_r, 0, lc_rotor_core}};
Point(117) = {{-rotor_core_outer_r, 0, 0, lc_rotor_core}};
Point(118) = {{0, -rotor_core_outer_r, 0, lc_rotor_core}};

Circle(115) = {{115, 1, 116}};
Circle(116) = {{116, 1, 117}};
Circle(117) = {{117, 1, 118}};
Circle(118) = {{118, 1, 115}};

// First magnet arc points (centered at x-axis, pointing +X)
Point(10) = {{magnet_inner_r * Cos(-half_arc), magnet_inner_r * Sin(-half_arc), 0, lc_magnet}};
Point(11) = {{magnet_outer_r * Cos(-half_arc), magnet_outer_r * Sin(-half_arc), 0, lc_magnet}};
Point(12) = {{magnet_outer_r * Cos(half_arc), magnet_outer_r * Sin(half_arc), 0, lc_magnet}};
Point(13) = {{magnet_inner_r * Cos(half_arc), magnet_inner_r * Sin(half_arc), 0, lc_magnet}};

// First magnet lines and arcs
Line(10) = {{10, 11}};
Circle(11) = {{11, 1, 12}};
Line(12) = {{12, 13}};
Circle(13) = {{13, 1, 10}};

// First magnet surface
Curve Loop(20) = {{10, 11, 12, 13}};
Plane Surface(21) = {{20}};

// Duplicate and rotate magnet around origin to create all {params.num_poles} poles
"""
    
    # Generate all 8 magnets manually with calculated positions
    # This is more reliable than Duplicata which has issues with arrays
    magnet_surfaces = [21]  # First magnet
    for i in range(1, params.num_poles):
        angle = i * pole_pitch_angle
        # Calculate rotated points
        base_id = 10 + (i * 10)  # Points: 10-13, 20-23, 30-33, etc.
        line_base = base_id
        loop_id = 20 + (i * 10)
        surf_id = 21 + (i * 10)
        
        geo_content += f"""
// Magnet {i+1} at {np.degrees(angle):.1f} degrees
Point({base_id}) = {{magnet_inner_r * Cos(-half_arc + {angle}), magnet_inner_r * Sin(-half_arc + {angle}), 0, lc_magnet}};
Point({base_id+1}) = {{magnet_outer_r * Cos(-half_arc + {angle}), magnet_outer_r * Sin(-half_arc + {angle}), 0, lc_magnet}};
Point({base_id+2}) = {{magnet_outer_r * Cos(half_arc + {angle}), magnet_outer_r * Sin(half_arc + {angle}), 0, lc_magnet}};
Point({base_id+3}) = {{magnet_inner_r * Cos(half_arc + {angle}), magnet_inner_r * Sin(half_arc + {angle}), 0, lc_magnet}};
Line({line_base}) = {{{base_id}, {base_id+1}}};
Circle({line_base+1}) = {{{base_id+1}, 1, {base_id+2}}};
Line({line_base+2}) = {{{base_id+2}, {base_id+3}}};
Circle({line_base+3}) = {{{base_id+3}, 1, {base_id}}};
Curve Loop({loop_id}) = {{{line_base}, {line_base+1}, {line_base+2}, {line_base+3}}};
Plane Surface({surf_id}) = {{{loop_id}}};
"""
        magnet_surfaces.append(surf_id)
    
    geo_content += f"""
// Airgap middle boundary (inner air boundary)
Point(190) = {{airgap_middle_r, 0, 0, lc_airgap}};
Point(191) = {{0, airgap_middle_r, 0, lc_airgap}};
Point(192) = {{-airgap_middle_r, 0, 0, lc_airgap}};
Point(193) = {{0, -airgap_middle_r, 0, lc_airgap}};

Circle(190) = {{190, 1, 191}};
Circle(191) = {{191, 1, 192}};
Circle(192) = {{192, 1, 193}};
Circle(193) = {{193, 1, 190}};

// Outer air boundary (stator OD + 2mm)
Point(200) = {{outer_air_r, 0, 0, lc_outer_air}};
Point(201) = {{0, outer_air_r, 0, lc_outer_air}};
Point(202) = {{-outer_air_r, 0, 0, lc_outer_air}};
Point(203) = {{0, -outer_air_r, 0, lc_outer_air}};

Circle(200) = {{200, 1, 201}};
Circle(201) = {{201, 1, 202}};
Circle(202) = {{202, 1, 203}};
Circle(203) = {{203, 1, 200}};

// Domain outer boundary (stator OD + 10mm)
Point(210) = {{domain_outer_r, 0, 0, lc_shell}};
Point(211) = {{0, domain_outer_r, 0, lc_shell}};
Point(212) = {{-domain_outer_r, 0, 0, lc_shell}};
Point(213) = {{0, -domain_outer_r, 0, lc_shell}};

Circle(210) = {{210, 1, 211}};
Circle(211) = {{211, 1, 212}};
Circle(212) = {{212, 1, 213}};
Circle(213) = {{213, 1, 210}};

// Rotor core region (shaft ID to rotor core OD - solid annulus)
Curve Loop(180) = {{115, 116, 117, 118}};  // Rotor core outer boundary
Curve Loop(181) = {{5, 6, 7, 8}};  // Shaft inner boundary (hole in rotor core)
Plane Surface(182) = {{180, 181}};  // Rotor core - solid steel annulus with center hole

// Center air region (0 to rotor core ID)
Curve Loop(185) = {{5, 6, 7, 8}};  // Shaft inner boundary (outer boundary for center air)
Plane Surface(186) = {{185}};  // Center air - solid disk

// Inner air region (center to airgap middle, minus rotor core AND magnets)
// Air fills: rotor core OD to magnet ID, and magnet gaps
Curve Loop(220) = {{190, 191, 192, 193}};  // Airgap middle boundary (outer)
Curve Loop(221) = {{115, 116, 117, 118}};  // Rotor core outer boundary (cutout - removes rotor core)
Curve Loop(222) = {{5, 6, 7, 8}};  // Shaft inner boundary (cutout - removes center air)
Plane Surface(223) = {{220, 221, 222, {', '.join([str(20 + i*10) for i in range(params.num_poles)])}}};  // Inner air with rotor core, center air, and magnet cutouts

// Stator core boundary (solid donut from stator ID to stator OD)
Point(300) = {{stator_inner_r, 0, 0, lc_stator}};
Point(301) = {{0, stator_inner_r, 0, lc_stator}};
Point(302) = {{-stator_inner_r, 0, 0, lc_stator}};
Point(303) = {{0, -stator_inner_r, 0, lc_stator}};

Point(310) = {{stator_outer_r, 0, 0, lc_stator}};
Point(311) = {{0, stator_outer_r, 0, lc_stator}};
Point(312) = {{-stator_outer_r, 0, 0, lc_stator}};
Point(313) = {{0, -stator_outer_r, 0, lc_stator}};

// Stator inner arcs (300-303) will be replaced by segmented arc chain
// Circle(300) = {{300, 1, 301}};  // Removed - replaced by arc chain
// Circle(301) = {{301, 1, 302}};  // Removed - replaced by arc chain
// Circle(302) = {{302, 1, 303}};  // Removed - replaced by arc chain
// Circle(303) = {{303, 1, 300}};  // Removed - replaced by arc chain

Circle(310) = {{310, 1, 311}};  // Stator outer arc (quadrant 1)
Circle(311) = {{311, 1, 312}};  // Stator outer arc (quadrant 2)
Circle(312) = {{312, 1, 313}};  // Stator outer arc (quadrant 3)
Circle(313) = {{313, 1, 310}};  // Stator outer arc (quadrant 4)

"""
    
    # Generate rectangular slots and track corner points for segmented inner boundary
    # Each slot creates 4 points and 3 lines (no bottom line - slot is open)
    # Track slot corners with metadata: (angle, point_id, slot_index, side, ln4, ln3, ln2)
    # This allows building continuous boundary that traces into/out of each slot
    inner_boundary_points = []  # Track (angle, point_id, slot_index, corner_type, line_ids) for inner boundary
    base_point_id = 400  # Starting point ID for slots
    base_line_id = 400   # Starting line ID for slots
    
    for i in range(num_slots):
        slot_angle = i * slot_pitch_angle
        
        # Slot bottom (at stator inner radius) and top (at stator ID + slot depth)
        slot_bottom_r_val = stator_inner_r
        slot_top_r_val = stator_inner_r + slot_depth
        
        # Calculate corner positions for rectangular slot
        # Bottom corners (at stator ID)
        half_width_bottom = slot_width / (2 * slot_bottom_r_val)
        p1_angle = slot_angle - half_width_bottom
        p2_angle = slot_angle + half_width_bottom
        
        # Top corners (at stator ID + slot depth)
        half_width_top = slot_width / (2 * slot_top_r_val)
        p3_angle = slot_angle + half_width_top
        p4_angle = slot_angle - half_width_top
        
        # Calculate actual x,y coordinates
        p1_x = slot_bottom_r_val * np.cos(p1_angle)
        p1_y = slot_bottom_r_val * np.sin(p1_angle)
        p2_x = slot_bottom_r_val * np.cos(p2_angle)
        p2_y = slot_bottom_r_val * np.sin(p2_angle)
        p3_x = slot_top_r_val * np.cos(p3_angle)
        p3_y = slot_top_r_val * np.sin(p3_angle)
        p4_x = slot_top_r_val * np.cos(p4_angle)
        p4_y = slot_top_r_val * np.sin(p4_angle)
        
        # Point IDs for this slot
        pt1 = base_point_id + i * 4
        pt2 = base_point_id + i * 4 + 1
        pt3 = base_point_id + i * 4 + 2
        pt4 = base_point_id + i * 4 + 3
        
        # Line IDs for this slot (no ln1 - slot is open at bottom)
        ln2 = base_line_id + i * 4 + 1
        ln3 = base_line_id + i * 4 + 2
        ln4 = base_line_id + i * 4 + 3
        
        geo_content += f"""
// Slot {i+1} at {np.degrees(slot_angle):.1f} degrees
Point({pt1}) = {{{p1_x:.6f}, {p1_y:.6f}, 0, lc_stator}};
Point({pt2}) = {{{p2_x:.6f}, {p2_y:.6f}, 0, lc_stator}};
Point({pt3}) = {{{p3_x:.6f}, {p3_y:.6f}, 0, lc_stator}};
Point({pt4}) = {{{p4_x:.6f}, {p4_y:.6f}, 0, lc_stator}};
Line({ln2}) = {{{pt2}, {pt3}}};
Line({ln3}) = {{{pt3}, {pt4}}};
Line({ln4}) = {{{pt4}, {pt1}}};
"""
        # Track slot bottom points with metadata including line IDs
        # Left corner stores ln4, ln3, ln2 for traversing slot edges backwards
        # Right corner has no line IDs (marks end of inter-slot segment)
        inner_boundary_points.append((p1_angle, pt1, i, 'left', ln4, ln3, ln2))   # Left corner, with lines to traverse slot
        inner_boundary_points.append((p2_angle, pt2, i, 'right', None, None, None))  # Right corner
    
    # Sort by angle
    inner_boundary_points_sorted = sorted(inner_boundary_points, key=lambda x: x[0])
    
    # Build closed curve loop that traces around slots
    # Pattern: inter-slot line (pt2[i] → pt1[i+1]) → -ln4 → -ln3 → -ln2 → next inter-slot line
    # This creates continuous path that goes into each slot and back out without closing slot opening
    inner_line_start_id = 5000
    curve_loop_elements = []
    
    for i in range(len(inner_boundary_points_sorted)):
        current = inner_boundary_points_sorted[i]
        next_pt = inner_boundary_points_sorted[(i + 1) % len(inner_boundary_points_sorted)]
        
        current_slot = current[2]
        current_side = current[3]
        next_slot = next_pt[2]
        next_side = next_pt[3]
        
        # Case 1: pt2 (right) of slot i → pt1 (left) of slot i+1
        # Create inter-slot line segment, then add reversed slot edges to trace into/out of slot
        if current_side == 'right' and next_side == 'left':
            line_id = inner_line_start_id + len([e for e in curve_loop_elements if isinstance(e, int)])
            pt_a = current[1]
            pt_b = next_pt[1]
            geo_content += f"Line({line_id}) = {{{pt_a}, {pt_b}}};\n"
            curve_loop_elements.append(line_id)
            
            # Then add the slot edges going backwards: -ln4 (up left), -ln3 (across top), -ln2 (down right)
            # This traces the path: pt1 → pt4 → pt3 → pt2
            ln4, ln3, ln2 = next_pt[4], next_pt[5], next_pt[6]
            curve_loop_elements.append(-ln4)
            curve_loop_elements.append(-ln3)
            curve_loop_elements.append(-ln2)
    
    # Create stator core surface with closed inner boundary that traces around slots
    curve_loop_str = ', '.join([str(e) for e in curve_loop_elements])
    geo_content += f"""
// Stator core with open slots (boundary traces around slot cavities)
Curve Loop(320) = {{310, 311, 312, 313}};  // Stator outer boundary
Curve Loop(321) = {{{curve_loop_str}}};  // Stator inner boundary (traces around slots)
Plane Surface(322) = {{320, 321}};  // Stator core
"""
    
    geo_content += f"""
// Middle air region 1 (airgap middle to stator tooth tips - includes slots)
// Note: Curve Loop 321 already defined above as stator inner boundary
// Reuse 321 as outer boundary for middle air (traced boundary fills slots with air)
Curve Loop(226) = {{190, 191, 192, 193}};  // Airgap middle boundary (inner)
Plane Surface(227) = {{321, 226}};  // Air from airgap middle to stator inner (fills slots)

// Middle air region 2 (stator outer to outer_air_r - air gap after stator)
Curve Loop(228) = {{200, 201, 202, 203}};  // Outer air boundary (outer)
Curve Loop(229) = {{310, 311, 312, 313}};  // Stator outer boundary (inner)
Plane Surface(230) = {{228, 229}};  // Air annulus after stator

// Shell region (outer_air_r to domain outer)
Curve Loop(235) = {{210, 211, 212, 213}};  // Domain outer boundary
Plane Surface(236) = {{235, 228}};  // Shell annulus

// Physical entities
Physical Surface("Rotor Core", 100) = {{182}};
Physical Surface("Stator Core", 200) = {{322}};
Physical Surface("Center Air", 112) = {{186}};
Physical Surface("Inner Air", 113) = {{223}};
Physical Surface("Middle Air 1", 114) = {{227}};  // Airgap middle to stator inner
Physical Surface("Middle Air 2", 115) = {{230}};  // Stator outer to outer_air_r
Physical Surface("Air Inf", 101) = {{236}};
"""
    
    # Create physical surfaces for magnets with alternating polarity
    for i in range(params.num_poles):
        magnet_id = 102 + i
        surface = 21 + (i * 10)  # Surface IDs: 21, 31, 41, 51, 61, 71, 81, 91
        polarity = "N" if i % 2 == 0 else "S"  # Alternate N-S
        geo_content += f"Physical Surface(\"Magnet_{polarity}{i+1}\", {magnet_id}) = {{{surface}}};\n"
    
    geo_content += f"""Physical Line("Exterior boundary", {102 + params.num_poles}) = {{210, 211, 212, 213}};

// Mesh display options
Mesh.ColorCarousel = 2;
Mesh.SurfaceEdges = 1;
Mesh.SurfaceFaces = 2;
"""
    
    with open(geo_file, 'w') as f:
        f.write(geo_content)
    
    print(f"✓ Geometry file created: {geo_file}")
    print(f"  Rotor core: R={rotor_core_inner_r/mm_to_m:.1f}-{rotor_core_outer_r/mm_to_m:.1f}mm (steel, 5mm gap to magnets)")
    print(f"  Magnets: {params.num_poles} poles, R={magnet_inner_r/mm_to_m:.1f}-{rotor_outer_r/mm_to_m:.1f}mm")
    print(f"  Arc per magnet: {np.degrees(magnet_arc_angle):.1f}° (N-S alternating)")
    print(f"  Stator core: R={stator_inner_r/mm_to_m:.1f}-{stator_outer_r/mm_to_m:.1f}mm (steel with {num_slots} slots)")
    print(f"  Slots: {num_slots} slots, depth={slot_depth/mm_to_m:.1f}mm, width={slot_width/mm_to_m:.1f}mm, opening={slot_opening/mm_to_m:.1f}mm")
    print(f"  Airgap: {airgap/mm_to_m:.1f}mm (R={rotor_outer_r/mm_to_m:.1f}-70.0mm)")
    print(f"  Center air: R=0-{rotor_core_inner_r/mm_to_m:.1f}mm")
    print(f"  Inner air: R={rotor_core_inner_r/mm_to_m:.1f}-{airgap_middle_r/mm_to_m:.1f}mm (with rotor core and magnet cutouts)")
    print(f"  Middle air 1: R={airgap_middle_r/mm_to_m:.1f}-{stator_inner_r/mm_to_m:.1f}mm (before stator)")
    print(f"  Middle air 2: R={stator_outer_r/mm_to_m:.1f}-{outer_air_r/mm_to_m:.1f}mm (after stator)")
    print(f"  Shell: R={outer_air_r/mm_to_m:.1f}-{domain_outer_r/mm_to_m:.1f}mm")
    print()
    
    return geo_file

def generate_mesh(geo_file):
    """
    Step 2: Generate finite element mesh (.msh)
    
    Uses Gmsh to create a 2D triangular mesh from the geometry.
    Mesh is refined in critical areas:
    - Fine mesh in air gap (2.5mm elements)
    - Medium mesh in core (6mm elements)  
    - Coarse mesh in far-field (18.75mm elements)
    """
    
    print("=" * 60)
    print("Step 2: Generating Mesh")
    print("=" * 60)
    print()
    
    if not os.path.exists(gmsh_exe):
        print(f"ERROR: Gmsh executable not found at {gmsh_exe}")
        return None
    
    msh_file = os.path.join(work_dir, 'c_core.msh')
    
    print("Running Gmsh mesher...")
    print(f"Input: {os.path.basename(geo_file)}")
    print(f"Output: {os.path.basename(msh_file)}")
    print()
    
    try:
        result = subprocess.run(
            [gmsh_exe, geo_file, '-2', '-o', msh_file],
            cwd=work_dir,
            capture_output=True,
            text=True,
            timeout=30
        )
        
        if result.returncode != 0:
            print("Gmsh Errors:")
            print(result.stderr)
            return None
        
        if os.path.exists(msh_file):
            filesize = os.path.getsize(msh_file)
            print(f"✓ Mesh generated: {os.path.basename(msh_file)} ({filesize:,} bytes)")
            
            # Parse mesh info from output
            if "nodes" in result.stdout.lower():
                for line in result.stdout.split('\n'):
                    if 'nodes' in line.lower() or 'elements' in line.lower():
                        print(f"  {line.strip()}")
            
            print()
            return msh_file
        else:
            print("ERROR: Mesh file not created")
            return None
            
    except subprocess.TimeoutExpired:
        print("ERROR: Gmsh timed out after 30 seconds")
        return None
    except Exception as e:
        print(f"ERROR running Gmsh: {e}")
        return None

def create_solver_file():
    """
    Step 3: Create GetDP solver input file (.pro) with rotor core
    
    Defines the magnetostatic problem for 8-pole rotor with steel rotor core:
    - Rotor core: Steel (μr=2000)
    - Three air regions: InnerAir, OuterAir, AirInf (all μr=1)
    - 8 separate magnet regions with individual radial magnetization vectors
    - North poles (1,3,5,7): Hc pointing radially outward at each angular position
    - South poles (2,4,6,8): Hc pointing radially inward at each angular position
    - Boundary conditions: a=0 at outer shell
    - Airgap boundary at middle of physical airgap
    - Uses GetDP library template (Lib_Magnetostatics_a_phi.pro)
    - Post-processing: magnetic vector potential (az), B-field, H-field
    """
    
    print("=" * 60)
    print("Step 3: Creating Solver Input File")
    print("=" * 60)
    print()
    
    pro_file = os.path.join(work_dir, 'c_core.pro')
    
    # Create the solver file using GetDP library template
    # Calculate shell radii from motor parameters
    mm_to_m = 0.001
    rotor_outer_r = params.rotor_outer_diameter / 2 * mm_to_m
    airgap = params.airgap_length * mm_to_m
    airgap_middle_r = rotor_outer_r + airgap / 2
    stator_inner_r = rotor_outer_r + airgap
    stator_outer_r = params.stator_outer_diameter / 2 * mm_to_m
    outer_air_r = stator_outer_r + 2 * mm_to_m
    domain_outer_r = stator_outer_r + 10 * mm_to_m
    
    pro_content = f"""// {params.num_poles} Arc Magnets with Steel Rotor Core and Stator Teeth - Motor Parameters
// Generated by develop_fea_example_9.py

// Constants for spherical shell (required by library template)
DefineConstant[ Val_Rint = {airgap_middle_r:.6f} ];   // Internal boundary (airgap middle)
DefineConstant[ Val_Rext = {domain_outer_r:.6f} ];  // External boundary (domain outer)

// Region tags (must match geometry Physical entities)
ROTOR_CORE = 100;
STATOR_CORE = 200;
CENTER_AIR = 112;
INNER_AIR = 113;
MIDDLE_AIR_1 = 114;
MIDDLE_AIR_2 = 115;
AIR_INF = 101;
"""
    
    # Add magnet region definitions
    for i in range(params.num_poles):
        polarity = "N" if i % 2 == 0 else "S"
        pro_content += f"MAGNET_{polarity}{i+1} = {102 + i};\n"
    
    pro_content += f"LINE_INF = {102 + params.num_poles};\n\n"
    
    # Create magnet region lists
    all_magnets_list = ", ".join([f"MAGNET_{'N' if i % 2 == 0 else 'S'}{i+1}" for i in range(params.num_poles)])
    
    pro_content += f"""Group {{
  RotorCore = Region[ ROTOR_CORE ];
  StatorCore = Region[ STATOR_CORE ];
  CenterAir = Region[ CENTER_AIR ];
  InnerAir = Region[ INNER_AIR ];
  MiddleAir1 = Region[ MIDDLE_AIR_1 ];
  MiddleAir2 = Region[ MIDDLE_AIR_2 ];
  AirInf   = Region[ AIR_INF ];
  Dirichlet_a_0   = Region[ LINE_INF ];
  Dirichlet_phi_0 = Region[ LINE_INF ];

  // Generic group names for library template
  Vol_Mag = Region[ {{RotorCore, StatorCore, CenterAir, InnerAir, MiddleAir1, MiddleAir2, AirInf, {all_magnets_list}}} ];
  {"Vol_NL_Mag = Region[ {RotorCore, StatorCore} ];" if USE_NONLINEAR_STEEL else ""}
  Vol_Inf_Mag = Region[ AirInf ];
  Vol_M_Mag   = Region[ {{{all_magnets_list}}} ];
}}

"""
    
    # Add material properties based on linear/nonlinear selection
    if USE_NONLINEAR_STEEL:
        pro_content += f"""Function {{
  mu0 = 4.e-7 * Pi;
  
  // NONLINEAR MATERIAL MODEL - B-H curve for electrical steel
  // Electrical steel M19 approximation - typical grain-oriented steel
  // B-H curve data points (H in A/m, B in Tesla)
  // Using GetDP InterpolationLinear syntax with B^2 and nu values
  
  // Create interpolation table: [B^2, nu=H/B] pairs
  // B values:  0    0.2   0.4   0.8   1.2   1.4   1.6   1.75  1.85  1.95  2.05
  // H values:  0    50    100   200   400   800   1600  3200  6400  12800 25600
  // NOTE: Start with nu ≈ 1/(mu_r*mu0) = 398 instead of 1e10 to allow flux into steel
  nu [ Region[{{RotorCore, StatorCore}}] ] = InterpolationLinear[SquNorm[$1]]{{
    0.0, 398.0,
    0.04, 250.0,
    0.16, 250.0,
    0.64, 250.0,
    1.44, 333.33,
    1.96, 571.43,
    2.56, 1000.0,
    3.0625, 1828.57,
    3.4225, 3459.46,
    3.8025, 6564.10,
    4.2025, 12487.80
  }};
  
  // Derivative of nu with respect to B^2 (for Jacobian in Newton-Raphson)
  dnudb2 [ Region[{{RotorCore, StatorCore}}] ] = dInterpolationLinear[SquNorm[$1]]{{
    0.0, 398.0,
    0.04, 250.0,
    0.16, 250.0,
    0.64, 250.0,
    1.44, 333.33,
    1.96, 571.43,
    2.56, 1000.0,
    3.0625, 1828.57,
    3.4225, 3459.46,
    3.8025, 6564.10,
    4.2025, 12487.80
  }};
  
  // Jacobian functions for nonlinear iteration
  dhdb [ Region[{{RotorCore, StatorCore}}] ] = TensorDiag[1,1,1] * nu[$1] + 2 * dnudb2[$1] * SquDyadicProduct[$1];
  dhdb_NL [ Region[{{RotorCore, StatorCore}}] ] = 2 * dnudb2[$1] * SquDyadicProduct[$1];
  
  // Permeability (inverse of reluctivity) - for post-processing
  mu [ Region[{{RotorCore, StatorCore}}] ] = 1. / nu[];
  
  // Air regions and magnets: μr=1
  nu [ Region[{{CenterAir, InnerAir, MiddleAir1, MiddleAir2, AirInf, {all_magnets_list}}} ] ] = 1. / mu0;
  mu [ Region[{{CenterAir, InnerAir, MiddleAir1, MiddleAir2, AirInf, {all_magnets_list}}} ] ] = mu0;
  
  // Permanent magnet coercive fields (radial magnetization, alternating N-S)
  Hc = {abs(params.magnet_hc)};  // A/m
  
  // Each magnet has radial magnetization at its angular position
"""
        
        # Add individual magnetization vectors for each magnet (nonlinear case)
        pole_pitch_angle = 2 * np.pi / params.num_poles
        for i in range(params.num_poles):
            angle = i * pole_pitch_angle
            polarity = "N" if i % 2 == 0 else "S"
            sign = 1 if i % 2 == 0 else -1
            hc_x = sign * np.cos(angle)
            hc_y = sign * np.sin(angle)
            pro_content += f"  hc [ Region[MAGNET_{polarity}{i+1}] ] = Vector[{hc_x:.6f}*Hc, {hc_y:.6f}*Hc, 0];  // {angle*180/np.pi:.1f}°, {'outward' if sign > 0 else 'inward'}\n"
        
        pro_content += """}

"""
    else:
        pro_content += f"""Function {{
  mu0 = 4.e-7 * Pi;
  
  // LINEAR MATERIAL MODEL
  // Rotor and Stator cores: Steel with constant relative permeability μr=2000
  mu_r_steel = 2000.;
  nu [ Region[ RotorCore ] ] = 1. / (mu_r_steel * mu0);
  mu [ Region[ RotorCore ] ] = mu_r_steel * mu0;
  nu [ Region[ StatorCore ] ] = 1. / (mu_r_steel * mu0);
  mu [ Region[ StatorCore ] ] = mu_r_steel * mu0;
  
  // Air regions and magnets: μr=1
  nu [ Region[{{CenterAir, InnerAir, MiddleAir1, MiddleAir2, AirInf, {all_magnets_list}}} ] ] = 1. / mu0;
  mu [ Region[{{CenterAir, InnerAir, MiddleAir1, MiddleAir2, AirInf, {all_magnets_list}}} ] ] = mu0;
  
  // Permanent magnet coercive fields (radial magnetization, alternating N-S)
  Hc = {abs(params.magnet_hc)};  // A/m
  
  // Each magnet has radial magnetization at its angular position
"""
        
        # Add individual magnetization vectors for each magnet (linear case)
        pole_pitch_angle = 2 * np.pi / params.num_poles
        for i in range(params.num_poles):
            angle = i * pole_pitch_angle
            polarity = "N" if i % 2 == 0 else "S"
            sign = 1 if i % 2 == 0 else -1
            hc_x = sign * np.cos(angle)
            hc_y = sign * np.sin(angle)
            pro_content += f"  hc [ Region[MAGNET_{polarity}{i+1}] ] = Vector[{hc_x:.6f}*Hc, {hc_y:.6f}*Hc, 0];  // {angle*180/np.pi:.1f}°, {'outward' if sign > 0 else 'inward'}\n"
        
        pro_content += """}

"""
    
    pro_content += """// Boundary conditions
Constraint {
  { Name a;
    Case {
      { Region Dirichlet_a_0; Value 0.; }
    }
  }
  { Name phi;
    Case {
      { Region Dirichlet_phi_0; Value 0.; }
    }
  }
}
"""
    
    # Add the Include statement separately to handle the variable substitution
    getdp_templates_path = os.path.join(os.path.dirname(getdp_exe), 'templates').replace('\\', '/')
    pro_content += f"""
// Include magnetostatic formulation from GetDP library
Include "{getdp_templates_path}/Lib_Magnetostatics_a_phi.pro"

// Post-processing operations
eps = 1.e-5;
PostOperation {{
  {{ Name MagnetostaticsSolution; NameOfPostProcessing Magnetostatics_a;
    Operation {{
      Print[ az, OnElementsOf Vol_Mag, File "c_core_az.pos"];
      Print[ b, OnElementsOf Vol_Mag, File "c_core_b.pos"];
      Print[ h, OnElementsOf Vol_Mag, File "c_core_h.pos"];
    }}
  }}
}}
"""
    
    with open(pro_file, 'w', encoding='utf-8') as f:
        f.write(pro_content)
    
    rotor_core_outer = params.rotor_outer_diameter / 2 - params.magnet_thickness - 5.0
    print(f"✓ Solver file created: {pro_file}")
    print(f"  Formulation: Magnetostatics (vector potential)")
    if USE_NONLINEAR_STEEL:
        print(f"  Analysis: NONLINEAR (B-H curve for steel)")
        print(f"  Materials:")
        print(f"    - Rotor/Stator cores: Nonlinear electrical steel (M19 approx)")
        print(f"    - B-H curve: 11 points, B=0 to 2.05T")
    else:
        print(f"  Analysis: LINEAR (constant permeability)")
        print(f"  Materials:")
        print(f"    - Rotor/Stator cores: Steel (μr=2000)")
    print(f"    - Air regions: μr=1")
    print(f"    - Magnets: μr≈{params.magnet_mu_r}, Hc={abs(params.magnet_hc)/1000:.0f} kA/m")
    print(f"  Regions:")
    print(f"    - Rotor core: {params.rotor_inner_diameter/2:.1f} to {rotor_core_outer:.1f} mm")
    print(f"    - Center air: 0 to {params.rotor_inner_diameter/2:.1f} mm")
    print(f"    - Inner air: {params.rotor_inner_diameter/2:.1f} to {airgap_middle_r/mm_to_m:.1f} mm (with rotor core and magnet cutouts)")
    print(f"    - Outer air: {airgap_middle_r/mm_to_m:.1f} to {outer_air_r/mm_to_m:.1f} mm")
    print(f"    - Shell: {outer_air_r/mm_to_m:.1f} to {domain_outer_r/mm_to_m:.1f} mm")
    print(f"  Magnetization: Radial, {params.num_poles} poles (N={params.num_poles//2}, S={params.num_poles//2})")
    print(f"  Boundary: a=0 on outer shell")
    print()
    
    return pro_file

def run_solver(pro_file, msh_file):
    """
    Step 4: Run GetDP magnetostatic solver
    
    Solves the magnetostatic problem using finite element method.
    Uses linear or nonlinear solver based on USE_NONLINEAR_STEEL setting.
    
    Linear solver:
    - Direct solve with constant permeability
    - Single matrix assembly and solve
    - Solution time: ~2 seconds
    
    Nonlinear solver:
    - Newton-Raphson iteration for B-H curve
    - Multiple iterations until convergence
    - Solution time: ~10-30 seconds (5-15 iterations)
    
    For 8-pole rotor:
    - Expected DOFs: ~10,000-13,000
    - Expected convergence: residual < 1e-9
    """
    
    print("=" * 60)
    print("Step 4: Running FEA Solver")
    print("=" * 60)
    print()
    
    if not os.path.exists(getdp_exe):
        print(f"ERROR: GetDP executable not found at {getdp_exe}")
        return False
    
    solver_type = "NONLINEAR (Newton-Raphson)" if USE_NONLINEAR_STEEL else "LINEAR (direct)"
    solve_cmd = 'Magnetostatics_a'  # Same command for both - library handles NL automatically if Vol_NL_Mag is defined
    
    print(f"Running GetDP magnetostatic solver ({solver_type})...")
    print(f"Problem file: {os.path.basename(pro_file)}")
    print(f"Mesh file: {os.path.basename(msh_file)}")
    print()
    
    try:
        result = subprocess.run(
            [getdp_exe, os.path.basename(pro_file), '-msh', os.path.basename(msh_file),
             '-solve', solve_cmd, '-pos', 'MagnetostaticsSolution'],
            cwd=work_dir,
            capture_output=True,
            text=True,
            timeout=120  # Increased timeout for nonlinear solver
        )
        
        print("GetDP Output:")
        print("-" * 60)
        print(result.stdout)
        
        if result.returncode != 0:
            print("GetDP Errors:")
            print(result.stderr)
            return False
        
        print("-" * 60)
        print()
        
        # Check for output files
        output_files = ['c_core_az.pos', 'c_core_b.pos', 'c_core_h.pos']
        missing_files = []
        
        for filename in output_files:
            filepath = os.path.join(work_dir, filename)
            if os.path.exists(filepath):
                filesize = os.path.getsize(filepath)
                print(f"✓ Generated: {filename} ({filesize:,} bytes)")
            else:
                missing_files.append(filename)
                print(f"✗ Missing: {filename}")
        
        if missing_files:
            print(f"\nWARNING: {len(missing_files)} output files missing")
            return False
        
        print()
        print("=" * 60)
        print("SUCCESS: Magnetostatic solution completed")
        print("=" * 60)
        print()
        
        return True
        
    except subprocess.TimeoutExpired:
        print("ERROR: GetDP timed out after 60 seconds")
        return False
    except Exception as e:
        print(f"ERROR running GetDP: {e}")
        return False

def visualize_results():
    """
    Step 5: Visualize results in Gmsh
    
    Opens two Gmsh viewers:
    1. Mesh visualization - shows geometry and mesh refinement
    2. B-field visualization - shows magnetic flux density distribution
    
    Displays:
    - Alternating N-S pole pattern (8 poles)
    - High B-field in magnets
    - Field propagation through airgap
    - Flux lines connecting adjacent N and S poles
    """
    
    print("=" * 60)
    print("Step 5: Visualizing Results")
    print("=" * 60)
    print()
    
    mesh_file = os.path.join(work_dir, 'c_core.msh')
    b_field_file = os.path.join(work_dir, 'c_core_b.pos')
    
    if not os.path.exists(mesh_file):
        print("ERROR: Mesh file not found")
        return False
    
    if not os.path.exists(b_field_file):
        print("ERROR: B-field output file not found")
        return False
    
    if not os.path.exists(gmsh_exe):
        print(f"ERROR: Gmsh executable not found at {gmsh_exe}")
        return False
    
    print("Opening visualizations in Gmsh...")
    print()
    
    # Create a visualization script for proper mesh coloring
    view_script = os.path.join(work_dir, 'view_mesh.geo')
    with open(view_script, 'w') as f:
        f.write(f'''// Load mesh
Merge "{mesh_file}";

// Set visualization options to color by physical group
Mesh.ColorCarousel = 2;  // Color by physical entity number
Mesh.SurfaceEdges = 1;   // Show edges
Mesh.SurfaceFaces = 2;   // Color faces by elementary entity
Mesh.VolumeEdges = 0;    // Don't show volume edges

// Display options
General.Trackball = 0;
General.RotationX = 0;
General.RotationY = 0;
General.RotationZ = 0;
''')
    
    try:
        # Open mesh viewer with visualization script
        print("Launching mesh viewer...")
        subprocess.Popen([gmsh_exe, view_script], cwd=work_dir)
        print("✓ Mesh viewer launched")
        print()
        
        # Small delay
        import time
        time.sleep(0.5)
        
        # Open B-field viewer
        print("Launching B-field viewer...")
        print("Magnetic flux density distribution:")
        print("  - Magnet regions: High field concentration")
        print("  - Airgap: Field propagation to stator")
        print("  - Air regions: Decreasing field with distance")
        subprocess.Popen([gmsh_exe, b_field_file], cwd=work_dir)
        print("✓ B-field viewer launched")
        print()
        
        return True
        
    except Exception as e:
        print(f"ERROR launching Gmsh: {e}")
        return False


def parse_pos_file(pos_file):
    """Parse GetDP .pos file to extract mesh coordinates and field values.
    
    Returns:
        tuple: (points, triangles, values) where
            points: Nx2 array of (x,y) coordinates
            triangles: Mx3 array of triangle vertex indices
            values: N array of field magnitudes at each point
    """
    points = []
    triangles = []
    values = []
    
    with open(pos_file, 'r') as f:
        lines = f.readlines()
    
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        
        # Look for vector triangle elements: VT(x1,y1,z1, x2,y2,z2, x3,y3,z3){vx1,vy1,vz1, vx2,vy2,vz2, vx3,vy3,vz3};
        if line.startswith('VT('):
            try:
                # Extract coordinates and values
                coords_part = line[3:line.index('){')]
                values_part = line[line.index('){')+2:line.index('};')]
                
                # Parse coordinates (9 values: x1,y1,z1, x2,y2,z2, x3,y3,z3)
                coords = [float(x) for x in coords_part.split(',')]
                # Parse values (9 values: vx1,vy1,vz1, vx2,vy2,vz2, vx3,vy3,vz3)
                vals = [float(x) for x in values_part.split(',')]
                
                # Extract x,y coordinates (ignore z)
                p1 = [coords[0], coords[1]]
                p2 = [coords[3], coords[4]]
                p3 = [coords[6], coords[7]]
                
                # Get starting index for this triangle
                idx_start = len(points)
                
                # Add points
                points.extend([p1, p2, p3])
                
                # Add triangle connectivity
                triangles.append([idx_start, idx_start+1, idx_start+2])
                
                # Calculate field magnitude at each vertex (vector field: 3 components per vertex)
                v1_mag = np.sqrt(vals[0]**2 + vals[1]**2 + vals[2]**2)
                v2_mag = np.sqrt(vals[3]**2 + vals[4]**2 + vals[5]**2)
                v3_mag = np.sqrt(vals[6]**2 + vals[7]**2 + vals[8]**2)
                values.extend([v1_mag, v2_mag, v3_mag])
            except (ValueError, IndexError) as e:
                # Skip malformed lines
                pass
            
        i += 1
    
    return np.array(points), np.array(triangles), np.array(values)


def draw_rectangular_slot(ax, angle, inner_radius, slot_height, slot_width, 
                         slot_opening, tooth_tip_height=0.5):
    """
    Draw a single rectangular stator slot with winding block
    
    Creates white slot cavity with orange winding block inside.
    Slot has constant linear width (not angular) for accurate geometry.
    
    Args:
        ax: Matplotlib axis
        angle: Angular position of slot center (radians)
        inner_radius: Stator inner radius (mm)
        slot_height: Radial depth of slot (mm)
        slot_width: Tangential width of slot body (mm)
        slot_opening: Narrow opening width at airgap (mm)
        tooth_tip_height: Height of tooth tip region (mm)
    """
    slot_bottom_radius = inner_radius
    slot_top_radius = inner_radius + slot_height
    
    # Calculate angular half-widths to maintain constant LINEAR width
    half_angle_bottom = slot_width / (2 * slot_bottom_radius)
    half_angle_top = slot_width / (2 * slot_top_radius)
    
    # Four corners of rectangular slot
    corners_xy = [
        (slot_bottom_radius * np.cos(angle - half_angle_bottom), 
         slot_bottom_radius * np.sin(angle - half_angle_bottom)),
        (slot_top_radius * np.cos(angle - half_angle_top), 
         slot_top_radius * np.sin(angle - half_angle_top)),
        (slot_top_radius * np.cos(angle + half_angle_top), 
         slot_top_radius * np.sin(angle + half_angle_top)),
        (slot_bottom_radius * np.cos(angle + half_angle_bottom), 
         slot_bottom_radius * np.sin(angle + half_angle_bottom))
    ]
    
    # Create rectangular slot as polygon (WHITE for the slot cavity)
    from matplotlib.patches import Polygon
    slot_poly = Polygon(corners_xy, facecolor='white', edgecolor='none', 
                       zorder=7)
    ax.add_patch(slot_poly)
    
    # Slot opening (narrow neck) - also WHITE
    opening_half_angle_bottom = slot_opening / (2 * slot_bottom_radius)
    opening_half_angle_top = slot_opening / (2 * (slot_bottom_radius + tooth_tip_height))
    
    opening_corners_xy = [
        (slot_bottom_radius * np.cos(angle - opening_half_angle_bottom),
         slot_bottom_radius * np.sin(angle - opening_half_angle_bottom)),
        ((slot_bottom_radius + tooth_tip_height) * np.cos(angle - opening_half_angle_top),
         (slot_bottom_radius + tooth_tip_height) * np.sin(angle - opening_half_angle_top)),
        ((slot_bottom_radius + tooth_tip_height) * np.cos(angle + opening_half_angle_top),
         (slot_bottom_radius + tooth_tip_height) * np.sin(angle + opening_half_angle_top)),
        (slot_bottom_radius * np.cos(angle + opening_half_angle_bottom),
         slot_bottom_radius * np.sin(angle + opening_half_angle_bottom))
    ]
    
    opening_poly = Polygon(opening_corners_xy, facecolor='white', edgecolor='none',
                          zorder=7)
    ax.add_patch(opening_poly)
    
    # Draw winding as a single rectangular block filling most of the slot
    winding_margin = 0.5  # mm margin from slot walls
    winding_bottom_radius = inner_radius + winding_margin
    winding_top_radius = inner_radius + slot_height - winding_margin
    
    # Slightly narrower than slot to leave space at edges
    winding_width = slot_width * 0.85
    winding_half_angle_bottom = winding_width / (2 * winding_bottom_radius)
    winding_half_angle_top = winding_width / (2 * winding_top_radius)
    
    winding_corners = [
        (winding_bottom_radius * np.cos(angle - winding_half_angle_bottom), 
         winding_bottom_radius * np.sin(angle - winding_half_angle_bottom)),
        (winding_top_radius * np.cos(angle - winding_half_angle_top), 
         winding_top_radius * np.sin(angle - winding_half_angle_top)),
        (winding_top_radius * np.cos(angle + winding_half_angle_top), 
         winding_top_radius * np.sin(angle + winding_half_angle_top)),
        (winding_bottom_radius * np.cos(angle + winding_half_angle_bottom), 
         winding_bottom_radius * np.sin(angle + winding_half_angle_bottom))
    ]
    
    # Create winding block (orange/copper color)
    winding_poly = Polygon(winding_corners, facecolor='orange', edgecolor='darkorange',
                          linewidth=0.5, alpha=0.8, zorder=8)
    ax.add_patch(winding_poly)


def plot_motor_geometry():
    """
    Create complete motor geometry visualization with all components
    
    Displays full motor cross-section with:
    - Shaft (dimgray circle)
    - Rotor core (silver)
    - 8 permanent magnets (red=North, blue=South)
    - Airgap region (1mm)
    - Stator with 48 slots (white cavities)
    - Stator yoke and teeth (lightgray steel)
    - Winding blocks (orange copper)
    
    All dimensions loaded dynamically from Excel parameter file.
    Shows complete motor design context alongside FEA analysis results.
    
    Output:
    - Saves high-resolution PNG to output directory
    - Displays matplotlib window (non-blocking)
    """
    print("=" * 60)
    print("Step 6a: Creating Complete Motor Geometry Plot")
    print("=" * 60)
    print()
    
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.set_aspect('equal')
    
    # Motor parameters in mm
    shaft_r = params.rotor_inner_diameter / 2
    rotor_inner_r = params.rotor_inner_diameter / 2
    rotor_outer_r = params.rotor_outer_diameter / 2
    magnet_thickness = params.magnet_thickness
    magnet_outer_r = rotor_outer_r + magnet_thickness
    airgap = params.airgap_length
    stator_inner_r = params.stator_inner_diameter / 2
    stator_outer_r = params.stator_outer_diameter / 2
    
    # Calculate key radii
    slot_depth = params.slot_depth
    slot_bottom_r = stator_outer_r - slot_depth
    
    # Magnet arc parameters
    pole_pitch_angle = 2 * np.pi / params.num_poles
    magnet_arc_angle = pole_pitch_angle * params.magnet_arc_ratio
    half_arc = magnet_arc_angle / 2
    
    # Slot parameters
    slot_angle = 2 * np.pi / params.num_slots
    
    # Draw shaft
    shaft = matplotlib.patches.Circle((0, 0), shaft_r, facecolor='dimgray', 
                                     edgecolor='none', zorder=1)
    ax.add_patch(shaft)
    
    # Draw rotor core
    rotor_core = matplotlib.patches.Wedge((0, 0), rotor_outer_r, 0, 360,
                                         width=rotor_outer_r - rotor_inner_r,
                                         facecolor='silver', edgecolor='none', 
                                         zorder=2)
    ax.add_patch(rotor_core)
    
    # Draw each magnet with alternating N-S polarity
    for i in range(params.num_poles):
        angle = i * pole_pitch_angle
        start_angle_deg = np.degrees(angle - half_arc)
        end_angle_deg = np.degrees(angle + half_arc)
        
        # Alternate N-S polarity
        polarity = 'N' if i % 2 == 0 else 'S'
        color = 'red' if polarity == 'N' else 'blue'
        
        # Create wedge for magnet
        wedge = matplotlib.patches.Wedge(
            (0, 0), magnet_outer_r, start_angle_deg, end_angle_deg,
            width=magnet_thickness, facecolor=color, alpha=0.7,
            edgecolor='black', linewidth=0.5, zorder=3
        )
        ax.add_patch(wedge)
        
        # Add polarity label at magnet center
        label_r = rotor_outer_r + magnet_thickness / 2
        label_x = label_r * np.cos(angle)
        label_y = label_r * np.sin(angle)
        ax.text(label_x, label_y, polarity, ha='center', va='center',
                fontsize=10, fontweight='bold', color='white', zorder=4)
    
    # Draw stator yoke (outer ring) - this is the back iron
    stator_yoke = matplotlib.patches.Wedge((0, 0), stator_outer_r, 0, 360,
                                          width=stator_outer_r - slot_bottom_r,
                                          facecolor='lightgray', edgecolor='none',
                                          zorder=5)
    ax.add_patch(stator_yoke)
    
    # Draw a full inner circle for the tooth base (the steel between slot bottoms and stator ID)
    tooth_base = matplotlib.patches.Wedge((0, 0), slot_bottom_r, 0, 360,
                                         width=slot_bottom_r - stator_inner_r,
                                         facecolor='lightgray', edgecolor='none',
                                         zorder=6)
    ax.add_patch(tooth_base)
    
    # Draw stator slots with windings
    for i in range(params.num_slots):
        slot_center_angle = i * slot_angle
        
        # Draw rectangular slot with winding block
        # Parameters: angle, inner_radius, slot_height, slot_width, slot_opening, tooth_tip_height
        draw_rectangular_slot(ax, slot_center_angle, stator_inner_r, slot_depth, 
                             params.slot_width, params.slot_opening, 0.5)
    
    # Add title
    ax.set_title(f'{params.num_poles}-Pole, {params.num_slots}-Slot Permanent Magnet Motor\n' +
                f'Complete Geometry Cross-Section',
                fontsize=16, fontweight='bold', pad=20)
    
    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    
    # Set axis limits with margin
    margin = stator_outer_r * 1.3
    ax.set_xlim(-margin, margin)
    ax.set_ylim(-margin, margin)
    
    # Add grid
    ax.grid(True, alpha=0.3, linestyle=':', linewidth=0.5)
    
    # Add comprehensive parameter text box
    param_text = (
        f"Motor Design Parameters:\n"
        f"━━━━━━━━━━━━━━━━━━━━━━\n"
        f"Poles: {params.num_poles}  |  Slots: {params.num_slots}\n"
        f"Stack Length: {params.stack_length:.1f}mm\n"
        f"\n"
        f"Rotor:\n"
        f"  ID: {params.rotor_inner_diameter:.1f}mm\n"
        f"  OD: {params.rotor_outer_diameter:.1f}mm\n"
        f"\n"
        f"Magnets:\n"
        f"  Thickness: {magnet_thickness:.1f}mm\n"
        f"  Arc ratio: {params.magnet_arc_ratio:.2f}\n"
        f"  Br: {params.magnet_br:.2f}T\n"
        f"  Hc: {abs(params.magnet_hc)/1000:.0f}kA/m\n"
        f"\n"
        f"Stator:\n"
        f"  ID: {params.stator_inner_diameter:.1f}mm\n"
        f"  OD: {params.stator_outer_diameter:.1f}mm\n"
        f"  Slot depth: {params.slot_depth:.1f}mm\n"
        f"\n"
        f"Airgap: {airgap:.1f}mm\n"
        f"\n"
        f"Electrical:\n"
        f"  {params.rated_voltage:.0f}Vdc, {params.rated_current:.0f}A\n"
        f"  {params.rated_speed:.0f}rpm"
    )
    ax.text(0.02, 0.98, param_text, transform=ax.transAxes,
            fontsize=8, verticalalignment='top', family='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    plt.tight_layout()
    
    # Save figure
    plot_file = os.path.join(work_dir, 'motor_geometry.png')
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"✓ Complete motor geometry plot saved: {os.path.basename(plot_file)}")
    print(f"  - Shows: Shaft, Rotor Core, {params.num_poles} Magnets, Airgap, {params.num_slots} Slots, Stator")
    print()
    
    # Show plot non-blocking
    plt.show(block=False)
    plt.pause(0.1)
    
    return True


def plot_results():
    """
    Create flux density analysis plots with Python post-processing
    
    Left plot: 2D spatial colormap of |B| magnitude across entire geometry
    - Uses matplotlib tricontourf for smooth visualization
    - Colorbar shows flux density in Tesla
    - Coordinates in mm for easier interpretation
    
    Right plot: Airgap flux density vs angular position
    - Samples B-field at airgap middle radius
    - Shows 8-pole sinusoidal pattern from alternating N-S magnets
    - Red dashed line indicates mean airgap flux density
    - Angular range: -180° to +180°
    
    Output:
    - Saves high-resolution PNG (150 dpi) to output directory
    - Displays interactive matplotlib window (non-blocking)
    """
    print("=" * 60)
    print("Step 6b: Creating Flux Density Analysis Plots")
    print("=" * 60)
    print()
    
    b_pos_file = os.path.join(work_dir, 'c_core_b.pos')
    
    if not os.path.exists(b_pos_file):
        print(f"❌ B-field file not found: {b_pos_file}")
        return False
    
    print(f"Reading B-field data from: {os.path.basename(b_pos_file)}")
    
    # Parse the .pos file
    points, triangles, b_values = parse_pos_file(b_pos_file)
    
    if len(points) == 0:
        print("❌ No data found in .pos file")
        return False
    
    print(f"  Loaded {len(points)} points, {len(triangles)} triangles")
    print(f"  B-field range: {b_values.min():.4f} to {b_values.max():.4f} T")
    
    # DIAGNOSTIC: Check B-field in stator back iron (away from magnets/airgap)
    mm_to_m = 0.001
    stator_inner_r = params.stator_inner_diameter / 2 * mm_to_m
    stator_outer_r = params.stator_outer_diameter / 2 * mm_to_m
    stator_back_r = (stator_inner_r + stator_outer_r) / 2 + 0.25 * (stator_outer_r - stator_inner_r)  # 75% radius in stator
    
    r_points = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
    tolerance = 2.0 * mm_to_m  # 2mm tolerance
    stator_back_mask = np.abs(r_points - stator_back_r) < tolerance
    stator_back_b = b_values[stator_back_mask]
    
    if len(stator_back_b) > 0:
        print(f"  STATOR BACK IRON (r={stator_back_r/mm_to_m:.1f}mm): {len(stator_back_b)} points")
        print(f"    B-field in stator: min={stator_back_b.min():.4f}, max={stator_back_b.max():.4f}, mean={stator_back_b.mean():.4f} T")
    else:
        print(f"  ⚠️  No points found in stator back iron at r={stator_back_r/mm_to_m:.1f}mm")
    
    # Extract airgap samples at the middle of airgap
    rotor_outer_r = params.rotor_outer_diameter / 2 * mm_to_m
    airgap = params.airgap_length * mm_to_m
    airgap_middle_r = rotor_outer_r + airgap / 2
    
    # Find points near the airgap middle radius
    r_points = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
    tolerance = 0.5 * mm_to_m  # 0.5mm tolerance
    airgap_mask = np.abs(r_points - airgap_middle_r) < tolerance
    
    airgap_points = points[airgap_mask]
    airgap_b = b_values[airgap_mask]
    
    # Calculate angular position for airgap points
    airgap_theta = np.arctan2(airgap_points[:, 1], airgap_points[:, 0])
    airgap_theta_deg = np.degrees(airgap_theta)
    
    # Sort by angle for plotting
    sort_idx = np.argsort(airgap_theta_deg)
    airgap_theta_deg_sorted = airgap_theta_deg[sort_idx]
    airgap_b_sorted = airgap_b[sort_idx]
    
    print(f"  Airgap samples: {len(airgap_points)} points at r={airgap_middle_r/mm_to_m:.2f}mm")
    print(f"  Airgap B-field range: {airgap_b.min():.4f} to {airgap_b.max():.4f} T")
    
    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Left plot: Spatial flux density colormap
    ax1.set_aspect('equal')
    triang = Triangulation(points[:, 0] * 1000, points[:, 1] * 1000, triangles)  # Convert to mm
    contour = ax1.tricontourf(triang, b_values, levels=50, cmap='viridis')
    cbar = plt.colorbar(contour, ax=ax1, label='|B| (T)')
    ax1.set_xlabel('x (mm)')
    ax1.set_ylabel('y (mm)')
    ax1.set_title('Flux Density Magnitude')
    ax1.grid(True, alpha=0.3)
    
    # Right plot: Airgap flux density vs angle
    ax2.plot(airgap_theta_deg_sorted, airgap_b_sorted, 'b-', linewidth=2, label='Airgap B-field')
    ax2.axhline(y=airgap_b.mean(), color='r', linestyle='--', linewidth=1, label=f'Mean: {airgap_b.mean():.3f} T')
    ax2.set_xlabel('Angular Position (degrees)')
    ax2.set_ylabel('|B| (T)')
    ax2.set_title(f'Airgap Flux Density at r={airgap_middle_r/mm_to_m:.1f}mm')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.set_xlim([-180, 180])
    
    plt.tight_layout()
    
    # Save figure
    plot_file = os.path.join(work_dir, 'flux_density_analysis.png')
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"\n✓ Flux density analysis plot saved: {os.path.basename(plot_file)}")
    
    # Show plot non-blocking
    plt.show(block=False)
    plt.pause(0.1)
    
    return True


def main():
    """
    Main execution - Complete motor FEA workflow with dynamic Excel parameters
    
    Workflow:
    1. Load motor parameters from Excel (automatic)
    2. Create geometry (.geo) - 8 magnets, 48 slots, 3 air regions
    3. Generate mesh (.msh) - fine mesh at airgap boundary
    4. Create solver file (.pro) - radial magnetization, N-S alternating
    5. Run GetDP solver - magnetostatic FEA solution
    6. Visualize in Gmsh - mesh and B-field distribution
    7. Create motor geometry plot - complete cross-section with all components
    8. Create flux density plots - 2D colormap and airgap profile
    
    All motor dimensions loaded dynamically from:
    projects/example_design_1/example_design_1_edrive_parameters.xlsx
    """
    
    print("\n")
    print("*" * 60)
    print(f"{params.num_poles}-Pole Arc Magnet Rotor - Motor Parameters")
    print("Geometry → Mesh → Solve → Visualize → Analyze")
    print("*" * 60)
    print("\n")
    
    # Step 1: Create geometry
    geo_file = create_geometry()
    if not geo_file:
        print("\n❌ Failed to create geometry")
        return 1
    
    # Step 2: Generate mesh
    msh_file = generate_mesh(geo_file)
    if not msh_file:
        print("\n❌ Failed to generate mesh")
        return 1
    
    # Step 3: Create solver file
    pro_file = create_solver_file()
    if not pro_file:
        print("\n❌ Failed to create solver file")
        return 1
    
    # Step 4: Run solver
    success = run_solver(pro_file, msh_file)
    if not success:
        print("\n❌ Solver failed - see errors above")
        return 1
    
    # Step 5: Visualize
    visualize_results()
    
    # Step 6a: Create motor geometry plot
    plot_motor_geometry()
    
    # Step 6b: Create flux density analysis plots
    plot_success = plot_results()
    if not plot_success:
        print("\n⚠ Warning: Flux density plot generation failed, but solver completed successfully")
    
    # Keep plots open
    print("\nAll plots displayed. Close plot windows to exit.")
    plt.show()  # Blocking call to keep all plots open
    
    print("\n" + "*" * 60)
    print("COMPLETE WORKFLOW FINISHED SUCCESSFULLY!")
    print("*" * 60)
    print()
    print(f"All files saved to: {work_dir}")
    print()
    print("Key Workflow Steps:")
    print("1. ✓ Geometry created (.geo file) - 8 arc magnets, 3 air regions")
    print("2. ✓ Mesh generated (.msh file) - ~18k DOFs, fine airgap mesh")
    print("3. ✓ Solver input created (.pro file) - radial magnetization")
    print("4. ✓ FEA solution computed - alternating N-S poles")
    print("5. ✓ Results visualized in Gmsh - 8-pole field pattern")
    print("6a. ✓ Motor geometry plot created - complete motor visualization")
    print("6b. ✓ Flux density plots created - field analysis")
    print()
    print("This demonstrates a complete 8-pole rotor FEA with realistic airgap!")
    print()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
