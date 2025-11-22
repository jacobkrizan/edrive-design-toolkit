"""develop_fea_example_15.py - Standalone Motor FEA with Rotation Study
========================================================================================

Complete FEA analysis tool for surface-mounted PM motors with distributed windings.
Standalone implementation with all parameters defined in-file (no external dependencies).

Key Features:
-------------

**Standalone Design:**
- All motor parameters defined directly in user configuration section
- No external parameter files or Excel imports required
- Clean, organized parameter structure with single definitions
- Only dependencies: NumPy, Matplotlib, gmsh, and getdp executables

**Rotor Rotation Study:**
- Parametric sweep of rotor mechanical angle for torque waveform analysis
- Synchronized current control: advances electrically with rotor position
- Formula: current_angle = WINDING_CURRENT_ANGLE + rotor_angle_elec
- Magnet Hc vectors rotate with geometry to maintain physical alignment
- FFT analysis for harmonic identification (6th, 12th for 48-slot, 8-pole)
- Enable/disable via ENABLE_ROTATION_STUDY flag

**Independent Excitation Control:**
- MAGNET_HC: Permanent magnet coercive field (A/m, 0 to disable)
- WINDING_CURRENT_DENSITY: Three-phase current density (A/m², 0 to disable)
- WINDING_CURRENT_ANGLE: Torque angle in electrical degrees (0°=d-axis, 90°=q-axis)

**Material Models:**
- Linear: Constant μr=2000 for steel cores
- Nonlinear: B-H curve (M19 electrical steel approximation)
- User-selectable via USE_NONLINEAR_STEEL flag

**Mesh Control:**
- Independent mesh sizing for all regions
- Critical airgap mesh control (default 0.5mm)
- Automatic refinement at stator tooth tips and slot openings

**Three-Phase Distributed Winding:**
- 48 slots, 8 poles (configurable)
- 120° electrical phase shift between phases
- Winding factor calculation via phasor summation
- Pattern: A+ A+ C- C- B+ B+ A- A- C+ C+ B- B- (repeats)

Post-Processing:
----------------
- Maxwell stress tensor torque calculation
- Flux linkage via airgap flux integration
- Motor constants: Kt (N-m/A), Ke (V/krpm)
- Complete geometry visualization
- 2D flux density colormaps
- Airgap flux profiles
- Torque waveform with FFT analysis
- All outputs saved to develop_fea_example_15_output/

Expected Results:
-----------------
- Torque: ~46-47 N-m (example motor with 5 A/mm² current density)
- Torque ripple: 1-5% (typical for distributed winding)
- Dominant harmonics: 6th and 12th (slot harmonics)
- No low-order harmonics when current synchronization is correct
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

# Paths to executables
getdp_dir = os.path.join(src_dir, 'getdp-3.5.0-Windows64')
getdp_exe = os.path.join(getdp_dir, 'getdp.exe')
getdp_templates = os.path.join(getdp_dir, 'templates')
gmsh_exe = os.path.join(src_dir, 'gmsh', 'gmsh.exe')

# Working directory for this example
work_dir = os.path.join(script_dir, 'develop_fea_example_15_output')

# ============================================================================
# USER CONFIGURATION
# ============================================================================

# --- MOTOR TOPOLOGY ---
NUM_POLES = 8
NUM_SLOTS = 48
TURNS_PER_COIL = 20          # Turns per coil (used in flux linkage calculation)

# --- MOTOR GEOMETRY (all dimensions in mm) ---
STATOR_OUTER_DIAMETER = 200.0
ROTOR_OUTER_DIAMETER = 138.0
ROTOR_INNER_DIAMETER = 50.0
AIRGAP_LENGTH = 1.0
STACK_LENGTH = 100.0

# Slot Geometry
SLOT_WIDTH = 4.0              # Tangential width
SLOT_HEIGHT = 12.0            # Radial depth  
SLOT_OPENING = 1.0            # Opening at airgap
WINDING_CLEARANCE = 0.5       # Insulation clearance

# Magnet Geometry
MAGNET_THICKNESS = 4.0        # Radial thickness
MAGNET_ARC_RATIO = 0.80       # Magnet arc / pole pitch

# Air Region
TOTAL_AIR_THICKNESS = 10.0    # Beyond stator OD

# --- EXCITATION CONTROL ---
MAGNET_HC = 1*900000.0            # A/m - Coercive field (set to 0 to disable PM)
WINDING_CURRENT_DENSITY = 0*5e6   # A/m² - Current density (set to 0 to disable)
WINDING_CURRENT_ANGLE = 0.0       # degrees - Electrical angle (0°=d-axis, 90°=q-axis)

# --- ANALYSIS SETTINGS ---
USE_NONLINEAR_STEEL = True        # True for B-H curve, False for linear μr=2000
ENABLE_ROTATION_STUDY = False     # True to compute torque waveform
ROTATION_STEPS = 6                # Number of rotor positions
ROTATION_START_DEG = 0.0          # Mechanical angle
ROTATION_END_DEG = 45.0           # Mechanical angle (45° = 1 elec cycle for 8-pole)

# --- MESH SETTINGS (all in mm) ---
MESH_ROTOR_CORE = 3.0
MESH_MAGNET = 2.0
MESH_AIRGAP = 0.5         # Critical for accuracy
MESH_STATOR_CORE = 3.0
MESH_WINDING = 2.0
MESH_AIR_GLOBAL = 10.0

# ============================================================================
# DERIVED PARAMETERS (Auto-calculated - DO NOT EDIT)
# ============================================================================
magnet_outer_r = ROTOR_OUTER_DIAMETER / 2 + MAGNET_THICKNESS
STATOR_INNER_DIAMETER = (magnet_outer_r + AIRGAP_LENGTH) * 2
POLE_PITCH_ANGLE = 2 * np.pi / NUM_POLES
SLOT_PITCH_ANGLE = 2 * np.pi / NUM_SLOTS
SLOTS_PER_POLE_PER_PHASE = NUM_SLOTS / (NUM_POLES * 3)

# ============================================================================

def assign_winding_phases(num_slots, num_poles):
    """
    Assign three-phase distributed winding pattern.
    
    For 48 slots, 8 poles (4 pole pairs):
    - Pattern repeats every 12 slots (one pole pair)
    - Each phase shifted by 4 slots (120 electrical degrees)
    - Creates proper rotating MMF
    
    Returns:
        list of tuples: (phase, polarity) for each slot
        phase: 'A', 'B', or 'C'
        polarity: +1 (forward current) or -1 (return current)
    """
    # Pattern for one pole pair (12 slots)
    # Format: (phase, polarity) for each pair of slots
    pole_pair_pattern = [
        ('A', +1),  # Slots 0-1
        ('C', -1),  # Slots 2-3
        ('B', +1),  # Slots 4-5
        ('A', -1),  # Slots 6-7
        ('C', +1),  # Slots 8-9
        ('B', -1),  # Slots 10-11
    ]
    
    winding_assignment = []
    for slot_idx in range(num_slots):
        # Position within pole pair (0 to 11)
        position_in_pole_pair = slot_idx % 12
        # Which pair of slots (0 to 5)
        pair_idx = position_in_pole_pair // 2
        
        phase, polarity = pole_pair_pattern[pair_idx]
        winding_assignment.append((phase, polarity))
    
    return winding_assignment

def create_geometry(rotor_angle_deg=0.0):
    """
    Step 1: Create the geometry file (.geo) with slotted stator
    
    Builds complete motor geometry with dynamic parameters from Excel:
    - 8 arc magnets on rotor surface (N-S alternating polarity)
    - Slotted stator with segmented inner boundary (traces around slot openings)
    - 48 rectangular slots with 3 edges each (no bottom edge across opening)
    - Inner boundary: inter-slot line segments + reversed slot edge traces
    - Air regions fill from airgap middle up to and including slot cavities
    - Fine mesh at airgap and in slots for accurate field calculation
    
    Args:
        rotor_angle_deg: Mechanical rotation angle of rotor in degrees (for rotation studies)
    
    All dimensions loaded dynamically from Excel parameter file.
    """
    
    print("=" * 60)
    print("Step 1: Creating Geometry")
    print("=" * 60)
    print()
    
    # Create output directory
    os.makedirs(work_dir, exist_ok=True)
    
    geo_file = os.path.join(work_dir, 'spm_motor.geo')
    
    # Calculate magnet geometry from motor parameters
    mm_to_m = 0.001
    rotor_outer_r = ROTOR_OUTER_DIAMETER / 2 * mm_to_m  # Magnet outer radius
    magnet_thickness = MAGNET_THICKNESS * mm_to_m
    magnet_inner_r = rotor_outer_r - magnet_thickness  # Magnet inner radius
    airgap = AIRGAP_LENGTH * mm_to_m  # Airgap between magnets and stator
    stator_inner_r = rotor_outer_r + airgap  # Stator inner radius
    airgap_middle_r = rotor_outer_r + airgap / 2  # Middle of airgap
    
    # Rotor core geometry - fits inside inner air region with small gap
    rotor_core_inner_r = ROTOR_INNER_DIAMETER / 2 * mm_to_m  # Shaft ID
    rotor_core_gap = 0.0001  # 0.1mm gap between rotor core OD and magnet inner surface
    rotor_core_outer_r = magnet_inner_r - rotor_core_gap  # Rotor core OD
    
    # Magnet arc angle (one pole)
    pole_pitch_angle = 2 * np.pi / NUM_POLES
    magnet_arc_angle = pole_pitch_angle * MAGNET_ARC_RATIO
    half_arc = magnet_arc_angle / 2
    
    # Rotor rotation offset (for rotation studies)
    rotor_angle_rad = np.radians(rotor_angle_deg)
    
    # Stator core geometry
    stator_outer_r = STATOR_OUTER_DIAMETER / 2 * mm_to_m
    
    # Slot geometry (rectangular slots)
    slot_depth = SLOT_HEIGHT * mm_to_m  # Radial depth of slot
    slot_width = SLOT_WIDTH * mm_to_m  # Tangential width of slot
    slot_opening = SLOT_OPENING * mm_to_m  # Opening width at airgap
    num_slots = NUM_SLOTS  # Number of slots
    slot_pitch_angle = 2 * np.pi / num_slots  # Angular spacing between slots
    
    # Boundary radii - air regions outside stator
    # Use user-defined total air thickness, split in middle
    total_air_thickness = TOTAL_AIR_THICKNESS * mm_to_m
    outer_air_r = stator_outer_r + total_air_thickness / 2  # Middle boundary
    domain_outer_r = stator_outer_r + total_air_thickness  # Outer boundary
    Val_Rint = airgap_middle_r   # Inner boundary at airgap middle
    Val_Rext = domain_outer_r   # Outer boundary at domain outer
    
    # Mesh sizes - convert from user settings (mm) to meters
    lc_rotor_core = MESH_ROTOR_CORE * mm_to_m
    lc_magnet = MESH_MAGNET * mm_to_m
    lc_airgap = MESH_AIRGAP * mm_to_m
    lc_stator = MESH_STATOR_CORE * mm_to_m
    lc_winding = MESH_WINDING * mm_to_m
    lc_air = MESH_AIR_GLOBAL * mm_to_m  # Used for all air regions
    
    geo_content = f"""// {NUM_POLES} Arc Magnets with Rotor Core and Stator Slots - Motor Parameters
// Generated by develop_fea_example_15.py

// Rotor core geometry (steel)
rotor_core_inner_r = {rotor_core_inner_r:.6f};  // {(rotor_core_inner_r/mm_to_m):.1f}mm (shaft ID)
rotor_core_outer_r = {rotor_core_outer_r:.6f};  // {(rotor_core_outer_r/mm_to_m):.1f}mm (rotor OD - 0.1mm gap)

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

// Mesh size parameters (user controlled)
lc_rotor_core = {lc_rotor_core:.6f};  // Rotor core: {MESH_ROTOR_CORE}mm
lc_magnet = {lc_magnet:.6f};  // Magnet: {MESH_MAGNET}mm
lc_airgap = {lc_airgap:.6f};  // Airgap: {MESH_AIRGAP}mm (critical for accuracy)
lc_stator = {lc_stator:.6f};  // Stator core: {MESH_STATOR_CORE}mm
lc_winding = {lc_winding:.6f};  // Windings: {MESH_WINDING}mm
lc_air = {lc_air:.6f};  // Air regions: {MESH_AIR_GLOBAL}mm

// Center point
Point(1) = {{0, 0, 0, lc_air}};

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

// First magnet arc points (with rotor rotation offset: {rotor_angle_deg:.3f}°)
Point(10) = {{magnet_inner_r * Cos(-half_arc + {rotor_angle_rad}), magnet_inner_r * Sin(-half_arc + {rotor_angle_rad}), 0, lc_magnet}};
Point(11) = {{magnet_outer_r * Cos(-half_arc + {rotor_angle_rad}), magnet_outer_r * Sin(-half_arc + {rotor_angle_rad}), 0, lc_magnet}};
Point(12) = {{magnet_outer_r * Cos(half_arc + {rotor_angle_rad}), magnet_outer_r * Sin(half_arc + {rotor_angle_rad}), 0, lc_magnet}};
Point(13) = {{magnet_inner_r * Cos(half_arc + {rotor_angle_rad}), magnet_inner_r * Sin(half_arc + {rotor_angle_rad}), 0, lc_magnet}};

// First magnet lines and arcs
Line(10) = {{10, 11}};
Circle(11) = {{11, 1, 12}};
Line(12) = {{12, 13}};
Circle(13) = {{13, 1, 10}};

// First magnet surface
Curve Loop(20) = {{10, 11, 12, 13}};
Plane Surface(21) = {{20}};

// Duplicate and rotate magnet around origin to create all {NUM_POLES} poles
"""
    
    # Generate all 8 magnets manually with calculated positions
    # This is more reliable than Duplicata which has issues with arrays
    magnet_surfaces = [21]  # First magnet
    for i in range(1, NUM_POLES):
        angle = i * pole_pitch_angle + rotor_angle_rad  # Add rotation offset
        # Calculate rotated points
        base_id = 10 + (i * 10)  # Points: 10-13, 20-23, 30-33, etc.
        line_base = base_id
        loop_id = 20 + (i * 10)
        surf_id = 21 + (i * 10)
        
        geo_content += f"""
// Magnet {i+1} at {np.degrees(angle):.1f} degrees (includes {rotor_angle_deg:.3f}° offset)
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

// Outer air boundary (middle between stator OD and domain outer)
Point(200) = {{outer_air_r, 0, 0, lc_air}};
Point(201) = {{0, outer_air_r, 0, lc_air}};
Point(202) = {{-outer_air_r, 0, 0, lc_air}};
Point(203) = {{0, -outer_air_r, 0, lc_air}};

Circle(200) = {{200, 1, 201}};
Circle(201) = {{201, 1, 202}};
Circle(202) = {{202, 1, 203}};
Circle(203) = {{203, 1, 200}};

// Domain outer boundary (total air thickness beyond stator OD)
Point(210) = {{domain_outer_r, 0, 0, lc_air}};
Point(211) = {{0, domain_outer_r, 0, lc_air}};
Point(212) = {{-domain_outer_r, 0, 0, lc_air}};
Point(213) = {{0, -domain_outer_r, 0, lc_air}};

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
Plane Surface(223) = {{220, 221, 222, {', '.join([str(20 + i*10) for i in range(NUM_POLES)])}}};  // Inner air with rotor core, center air, and magnet cutouts

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
        
        # Slot radii (bottom at stator inner, top at inner + depth)
        slot_bottom_r = stator_inner_r
        slot_top_r = stator_inner_r + slot_depth
        
        # Perfect rectangular slot: fixed angular positions for left/right edges
        # Use average radius to center the slot tangentially
        avg_slot_r = stator_inner_r + slot_depth / 2
        half_angular_width = slot_width / (2 * avg_slot_r)
        
        # Left and right angles (fixed for all corners)
        left_angle = slot_angle - half_angular_width
        right_angle = slot_angle + half_angular_width
        
        # Four corners: two fixed angles × two radii = perfect rectangle
        # p1 (bottom-left): left angle, bottom radius
        # p2 (bottom-right): right angle, bottom radius
        # p3 (top-right): right angle, top radius
        # p4 (top-left): left angle, top radius
        p1_x = slot_bottom_r * np.cos(left_angle)
        p1_y = slot_bottom_r * np.sin(left_angle)
        p2_x = slot_bottom_r * np.cos(right_angle)
        p2_y = slot_bottom_r * np.sin(right_angle)
        p3_x = slot_top_r * np.cos(right_angle)
        p3_y = slot_top_r * np.sin(right_angle)
        p4_x = slot_top_r * np.cos(left_angle)
        p4_y = slot_top_r * np.sin(left_angle)
        
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
Point({pt1}) = {{{p1_x:.6f}, {p1_y:.6f}, 0, lc_airgap}};  // Bottom left - use airgap mesh for tooth tip
Point({pt2}) = {{{p2_x:.6f}, {p2_y:.6f}, 0, lc_airgap}};  // Bottom right - use airgap mesh for tooth tip
Point({pt3}) = {{{p3_x:.6f}, {p3_y:.6f}, 0, lc_stator}};
Point({pt4}) = {{{p4_x:.6f}, {p4_y:.6f}, 0, lc_stator}};
Line({ln2}) = {{{pt2}, {pt3}}};
Line({ln3}) = {{{pt3}, {pt4}}};
Line({ln4}) = {{{pt4}, {pt1}}};
"""
        # Track slot bottom points with metadata including line IDs
        # Left corner stores ln4, ln3, ln2 for traversing slot edges backwards
        # Right corner has no line IDs (marks end of inter-slot segment)
        inner_boundary_points.append((left_angle, pt1, i, 'left', ln4, ln3, ln2))   # Left corner, with lines to traverse slot
        inner_boundary_points.append((right_angle, pt2, i, 'right', None, None, None))  # Right corner
    
    # Generate winding blocks inside slots (treated as air for Step 1)
    # Each winding is a perfect rectangle with uniform clearance on all sides
    winding_base_point_id = 6000
    winding_base_line_id = 6000
    winding_surface_ids = []
    clearance = WINDING_CLEARANCE * mm_to_m
    
    for i in range(num_slots):
        slot_angle = i * slot_pitch_angle
        
        # Winding dimensions: slot dimensions minus clearance on all sides
        winding_width = slot_width - 2 * clearance
        winding_height = slot_depth - 2 * clearance
        
        # Winding radii: inset by clearance from slot boundaries
        winding_bottom_r = stator_inner_r + clearance
        winding_top_r = stator_inner_r + slot_depth - clearance
        
        # Perfect rectangular winding: fixed angular positions for left/right edges
        # Use average winding radius to center tangentially
        avg_winding_r = (winding_bottom_r + winding_top_r) / 2
        half_angular_width_winding = winding_width / (2 * avg_winding_r)
        
        # Left and right angles (fixed for all winding corners)
        winding_left_angle = slot_angle - half_angular_width_winding
        winding_right_angle = slot_angle + half_angular_width_winding
        
        # Four corners: two fixed angles × two radii = perfect rectangle with uniform clearance
        # wp1 (bottom-left): left angle, bottom radius (inner + clearance)
        # wp2 (bottom-right): right angle, bottom radius
        # wp3 (top-right): right angle, top radius (inner + depth - clearance)
        # wp4 (top-left): left angle, top radius
        w1_x = winding_bottom_r * np.cos(winding_left_angle)
        w1_y = winding_bottom_r * np.sin(winding_left_angle)
        w2_x = winding_bottom_r * np.cos(winding_right_angle)
        w2_y = winding_bottom_r * np.sin(winding_right_angle)
        w3_x = winding_top_r * np.cos(winding_right_angle)
        w3_y = winding_top_r * np.sin(winding_right_angle)
        w4_x = winding_top_r * np.cos(winding_left_angle)
        w4_y = winding_top_r * np.sin(winding_left_angle)
        
        # Point IDs for winding block
        wp1 = winding_base_point_id + i * 4
        wp2 = winding_base_point_id + i * 4 + 1
        wp3 = winding_base_point_id + i * 4 + 2
        wp4 = winding_base_point_id + i * 4 + 3
        
        # Line IDs for winding block
        wl1 = winding_base_line_id + i * 4
        wl2 = winding_base_line_id + i * 4 + 1
        wl3 = winding_base_line_id + i * 4 + 2
        wl4 = winding_base_line_id + i * 4 + 3
        
        # Curve loop and surface IDs
        wcl = 7000 + i
        ws = 7000 + i
        winding_surface_ids.append(ws)
        
        geo_content += f"""
// Winding block in Slot {i+1} (treated as air initially, clearance={WINDING_CLEARANCE}mm on all sides)
Point({wp1}) = {{{w1_x:.6f}, {w1_y:.6f}, 0, lc_winding}};
Point({wp2}) = {{{w2_x:.6f}, {w2_y:.6f}, 0, lc_winding}};
Point({wp3}) = {{{w3_x:.6f}, {w3_y:.6f}, 0, lc_winding}};
Point({wp4}) = {{{w4_x:.6f}, {w4_y:.6f}, 0, lc_winding}};
Line({wl1}) = {{{wp1}, {wp2}}};
Line({wl2}) = {{{wp2}, {wp3}}};
Line({wl3}) = {{{wp3}, {wp4}}};
Line({wl4}) = {{{wp4}, {wp1}}};
Curve Loop({wcl}) = {{{wl1}, {wl2}, {wl3}, {wl4}}};
Plane Surface({ws}) = {{{wcl}}};
"""
    
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
    
    # Build winding curve loop list for middle air cutouts
    winding_curve_loops = ', '.join([str(7000 + i) for i in range(num_slots)])
    
    geo_content += f"""
// Middle air region 1 (airgap middle to stator tooth tips - slots with winding cutouts)
// Note: Curve Loop 321 already defined above as stator inner boundary
// Windings are cutouts from the air region (air fills slot space around windings)
Curve Loop(226) = {{190, 191, 192, 193}};  // Airgap middle boundary (inner)
Plane Surface(227) = {{321, 226, {winding_curve_loops}}};  // Air from airgap middle to stator inner, minus windings

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
Physical Surface("Middle Air 1", 114) = {{227}};  // Airgap middle to stator inner (air around windings)
Physical Surface("Middle Air 2", 115) = {{230}};  // Stator outer to outer_air_r
Physical Surface("Air Inf", 101) = {{236}};
"""
    
    # Create physical surfaces for windings (treated as air initially)
    for i in range(num_slots):
        winding_id = 500 + i
        surface = 7000 + i
        geo_content += f'Physical Surface("Winding Air Slot_{i+1}", {winding_id}) = {{{surface}}};\n'
    
    # Create physical surfaces for magnets with alternating polarity
    for i in range(NUM_POLES):
        magnet_id = 102 + i
        surface = 21 + (i * 10)  # Surface IDs: 21, 31, 41, 51, 61, 71, 81, 91
        polarity = "N" if i % 2 == 0 else "S"  # Alternate N-S
        geo_content += f"Physical Surface(\"Magnet_{polarity}{i+1}\", {magnet_id}) = {{{surface}}};\n"
    
    geo_content += f"""Physical Line("Exterior boundary", {102 + NUM_POLES}) = {{210, 211, 212, 213}};

// Mesh display options
Mesh.ColorCarousel = 2;
Mesh.SurfaceEdges = 1;
Mesh.SurfaceFaces = 2;
"""
    
    with open(geo_file, 'w') as f:
        f.write(geo_content)
    
    print(f"✓ Geometry file created: {geo_file}")
    print(f"  Rotor core: R={rotor_core_inner_r/mm_to_m:.1f}-{rotor_core_outer_r/mm_to_m:.1f}mm (steel, 0.1mm gap to magnets)")
    print(f"  Magnets: {NUM_POLES} poles, R={magnet_inner_r/mm_to_m:.1f}-{rotor_outer_r/mm_to_m:.1f}mm")
    print(f"  Arc per magnet: {np.degrees(magnet_arc_angle):.1f}° (N-S alternating)")
    print(f"  Stator core: R={stator_inner_r/mm_to_m:.1f}-{stator_outer_r/mm_to_m:.1f}mm (steel with {num_slots} slots)")
    print(f"  Slots: {num_slots} slots, depth={slot_depth/mm_to_m:.1f}mm, width={slot_width/mm_to_m:.1f}mm, opening={slot_opening/mm_to_m:.1f}mm")
    print(f"  Airgap: {airgap/mm_to_m:.1f}mm (R={rotor_outer_r/mm_to_m:.1f}-{stator_inner_r/mm_to_m:.1f}mm)")
    print(f"  Center air: R=0-{rotor_core_inner_r/mm_to_m:.1f}mm")
    print(f"  Inner air: R={rotor_core_inner_r/mm_to_m:.1f}-{airgap_middle_r/mm_to_m:.1f}mm (with rotor core and magnet cutouts)")
    print(f"  Middle air 1: R={airgap_middle_r/mm_to_m:.1f}-{stator_inner_r/mm_to_m:.1f}mm (before stator)")
    print(f"  Middle air 2: R={stator_outer_r/mm_to_m:.1f}-{outer_air_r/mm_to_m:.1f}mm (first half beyond stator)")
    print(f"  Shell: R={outer_air_r/mm_to_m:.1f}-{domain_outer_r/mm_to_m:.1f}mm (second half, total={TOTAL_AIR_THICKNESS:.1f}mm beyond stator)")
    print(f"  Mesh: rotor={MESH_ROTOR_CORE}mm, magnet={MESH_MAGNET}mm, airgap={MESH_AIRGAP}mm, stator={MESH_STATOR_CORE}mm, air={MESH_AIR_GLOBAL}mm")
    print()
    
    return geo_file

def generate_mesh(geo_file):
    """
    Step 2: Generate finite element mesh (.msh)
    
    Uses Gmsh to create a 2D triangular mesh from the geometry.
    Mesh sizes are user-controlled via settings at top of file:
    - Rotor core: MESH_ROTOR_CORE (default 5mm)
    - Magnets: MESH_MAGNET (default 1mm)
    - Airgap: MESH_AIRGAP (default 0.3mm, critical for accuracy)
    - Stator core: MESH_STATOR_CORE (default 5mm)
    - Air regions: MESH_AIR_GLOBAL (default 10mm)
    """
    
    print("=" * 60)
    print("Step 2: Generating Mesh")
    print("=" * 60)
    print()
    
    if not os.path.exists(gmsh_exe):
        print(f"ERROR: Gmsh executable not found at {gmsh_exe}")
        return None
    
    msh_file = os.path.join(work_dir, 'spm_motor.msh')
    
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

def create_solver_file(magnet_hc, winding_current_density, winding_current_angle, use_nonlinear_steel, rotor_angle_deg=0.0):
    """
    Step 3: Create GetDP solver input file (.pro) with rotor core
    
    Args:
        magnet_hc: Magnet coercive field in A/m (set to 0 to disable PM excitation)
        winding_current_density: Winding current density in A/m² (set to 0 to disable winding excitation)
        winding_current_angle: Electrical angle in degrees for torque/position analysis
        use_nonlinear_steel: True for nonlinear B-H curve, False for linear μr=2000
        rotor_angle_deg: Mechanical rotation angle of rotor (for Hc vector rotation)
    
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
    MAGNET_HC = magnet_hc
    WINDING_CURRENT_DENSITY = winding_current_density
    WINDING_CURRENT_ANGLE = winding_current_angle
    USE_NONLINEAR_STEEL = use_nonlinear_steel
    
    print("=" * 60)
    print("Step 3: Creating Solver Input File")
    print("=" * 60)
    print()
    
    pro_file = os.path.join(work_dir, 'spm_motor.pro')
    
    # Create the solver file using GetDP library template
    # Calculate shell radii from motor parameters
    mm_to_m = 0.001
    num_slots = NUM_SLOTS  # Get number of slots for winding definitions
    rotor_outer_r = ROTOR_OUTER_DIAMETER / 2 * mm_to_m
    airgap = AIRGAP_LENGTH * mm_to_m
    airgap_middle_r = rotor_outer_r + airgap / 2
    stator_inner_r = rotor_outer_r + airgap
    stator_outer_r = STATOR_OUTER_DIAMETER / 2 * mm_to_m
    outer_air_r = stator_outer_r + 2 * mm_to_m
    domain_outer_r = stator_outer_r + 10 * mm_to_m
    
    pro_content = f"""// {NUM_POLES} Arc Magnets with Steel Rotor Core and Stator Teeth - Motor Parameters
// Generated by develop_fea_example_15.py

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
    for i in range(NUM_POLES):
        polarity = "N" if i % 2 == 0 else "S"
        pro_content += f"MAGNET_{polarity}{i+1} = {102 + i};\n"
    
    pro_content += f"LINE_INF = {102 + NUM_POLES};\n"
    
    # Add winding region definitions (treated as air for Step 1)
    for i in range(num_slots):
        pro_content += f"WINDING_{i+1} = {500 + i};\n"
    
    pro_content += "\n"
    
    # Create magnet region lists
    all_magnets_list = ", ".join([f"MAGNET_{'N' if i % 2 == 0 else 'S'}{i+1}" for i in range(NUM_POLES)])
    
    # Create winding region list
    all_windings_list = ", ".join([f"WINDING_{i+1}" for i in range(num_slots)])
    
    # Get winding phase assignments for current excitation
    winding_phases = assign_winding_phases(num_slots, NUM_POLES)
    
    pro_content += f"""Group {{
  RotorCore = Region[ ROTOR_CORE ];
  StatorCore = Region[ STATOR_CORE ];
  CenterAir = Region[ CENTER_AIR ];
  InnerAir = Region[ INNER_AIR ];
  MiddleAir1 = Region[ MIDDLE_AIR_1 ];
  MiddleAir2 = Region[ MIDDLE_AIR_2 ];
  AirInf   = Region[ AIR_INF ];
  Windings = Region[ {{{all_windings_list}}} ];  // Three-phase distributed winding
  Dirichlet_a_0   = Region[ LINE_INF ];
  Dirichlet_phi_0 = Region[ LINE_INF ];

  // Generic group names for library template
  Vol_Mag = Region[ {{RotorCore, StatorCore, CenterAir, InnerAir, MiddleAir1, MiddleAir2, AirInf, Windings, {all_magnets_list}}} ];
  {"Vol_NL_Mag = Region[ {RotorCore, StatorCore} ];" if USE_NONLINEAR_STEEL else ""}
  Vol_Inf_Mag = Region[ AirInf ];
  Vol_M_Mag   = Region[ {{{all_magnets_list}}} ];
  Vol_S0_Mag = Region[ Windings ];  // Current source regions
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
  
  // Windings: μr=1 (non-magnetic conductor)
  nu [ Region[ Windings ] ] = 1. / mu0;
  mu [ Region[ Windings ] ] = mu0;
  
  // Permanent magnet coercive fields (radial magnetization, alternating N-S)
  Hc = {MAGNET_HC};  // A/m (set to 0 to disable PM excitation)
  
  // Each magnet has radial magnetization at its angular position
"""
        
        # Add individual magnetization vectors for each magnet (nonlinear case)
        # NOTE: Hc vectors must rotate with geometry to match magnet physical positions
        pole_pitch_angle = 2 * np.pi / NUM_POLES
        rotor_angle_rad = np.radians(rotor_angle_deg) if 'rotor_angle_deg' in locals() else 0.0
        for i in range(NUM_POLES):
            angle = i * pole_pitch_angle + rotor_angle_rad  # Rotate Hc with geometry
            polarity = "N" if i % 2 == 0 else "S"
            sign = 1 if i % 2 == 0 else -1
            hc_x = sign * np.cos(angle)
            hc_y = sign * np.sin(angle)
            pro_content += f"  hc [ Region[MAGNET_{polarity}{i+1}] ] = Vector[{hc_x:.6f}*Hc, {hc_y:.6f}*Hc, 0];  // {angle*180/np.pi:.1f}°, {'outward' if sign > 0 else 'inward'}\n"
        
        # Add three-phase current excitation
        pro_content += f"""
  
  // Three-phase winding current density (z-direction, perpendicular to 2D plane)
  J_density = {WINDING_CURRENT_DENSITY};  // A/m² (set to 0 to disable winding excitation)
  theta_elec = {np.radians(WINDING_CURRENT_ANGLE)};  // Electrical angle (radians)
  
  // Three-phase currents with 120° phase shift
  I_A = J_density * Cos[theta_elec];
  I_B = J_density * Cos[theta_elec - 2*Pi/3];
  I_C = J_density * Cos[theta_elec + 2*Pi/3];
  
  // Current density assignment for each winding slot (z-direction)
"""
        # Generate current density for each winding based on phase assignment
        for i in range(num_slots):
            phase, polarity = winding_phases[i]
            current_var = f"I_{phase}"
            sign_str = "+" if polarity > 0 else "-"
            pro_content += f"  js[ Region[WINDING_{i+1}] ] = Vector[0, 0, {sign_str}{current_var}];  // Slot {i+1}: Phase {phase}{'+' if polarity > 0 else '-'}\n"
        
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
  
  // Windings: μr=1 (non-magnetic conductor)
  nu [ Region[ Windings ] ] = 1. / mu0;
  mu [ Region[ Windings ] ] = mu0;
  
  // Permanent magnet coercive fields (radial magnetization, alternating N-S)
  Hc = {MAGNET_HC};  // A/m (set to 0 to disable PM excitation)
  
  // Each magnet has radial magnetization at its angular position
"""
        
        # Add individual magnetization vectors for each magnet (linear case)
        # NOTE: Hc vectors must rotate with geometry to match magnet physical positions
        pole_pitch_angle = 2 * np.pi / NUM_POLES
        rotor_angle_rad = np.radians(rotor_angle_deg) if 'rotor_angle_deg' in locals() else 0.0
        for i in range(NUM_POLES):
            angle = i * pole_pitch_angle + rotor_angle_rad  # Rotate Hc with geometry
            polarity = "N" if i % 2 == 0 else "S"
            sign = 1 if i % 2 == 0 else -1
            hc_x = sign * np.cos(angle)
            hc_y = sign * np.sin(angle)
            pro_content += f"  hc [ Region[MAGNET_{polarity}{i+1}] ] = Vector[{hc_x:.6f}*Hc, {hc_y:.6f}*Hc, 0];  // {angle*180/np.pi:.1f}°, {'outward' if sign > 0 else 'inward'}\n"
        
        # Add three-phase current excitation
        pro_content += f"""
  
  // Three-phase winding current density (z-direction, perpendicular to 2D plane)
  J_density = {WINDING_CURRENT_DENSITY};  // A/m² (set to 0 to disable winding excitation)
  theta_elec = {np.radians(WINDING_CURRENT_ANGLE)};  // Electrical angle (radians)
  
  // Three-phase currents with 120° phase shift
  I_A = J_density * Cos[theta_elec];
  I_B = J_density * Cos[theta_elec - 2*Pi/3];
  I_C = J_density * Cos[theta_elec + 2*Pi/3];
  
  // Current density assignment for each winding slot (z-direction)
"""
        # Generate current density for each winding based on phase assignment
        for i in range(num_slots):
            phase, polarity = winding_phases[i]
            current_var = f"I_{phase}"
            sign_str = "+" if polarity > 0 else "-"
            pro_content += f"  js[ Region[WINDING_{i+1}] ] = Vector[0, 0, {sign_str}{current_var}];  // Slot {i+1}: Phase {phase}{'+' if polarity > 0 else '-'}\n"
        
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
      Print[ az, OnElementsOf Vol_Mag, File "spm_motor_az.pos"];
      Print[ b, OnElementsOf Vol_Mag, File "spm_motor_b.pos"];
      Print[ h, OnElementsOf Vol_Mag, File "spm_motor_h.pos"];
    }}
  }}
}}
"""
    
    with open(pro_file, 'w', encoding='utf-8') as f:
        f.write(pro_content)
    
    rotor_core_outer = ROTOR_OUTER_DIAMETER / 2 - MAGNET_THICKNESS - 5.0
    print(f"✓ Solver file created: {pro_file}")
    print(f"  Formulation: Magnetostatics (vector potential)")
    
    # Display excitation settings
    excitation_parts = []
    if MAGNET_HC > 0:
        excitation_parts.append(f"PM (Hc={MAGNET_HC/1000:.0f} kA/m)")
    else:
        excitation_parts.append("PM disabled (Hc=0)")
    
    if WINDING_CURRENT_DENSITY > 0:
        excitation_parts.append(f"3-Phase Current ({WINDING_CURRENT_DENSITY/1e6:.1f} A/mm², θ={WINDING_CURRENT_ANGLE}°)")
    else:
        excitation_parts.append("Windings disabled (J=0)")
    
    print(f"  Excitation: {' | '.join(excitation_parts)}")
    
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
    print(f"    - Windings: μr=1")
    print(f"    - Magnets: μr≈{1.05}")
    print(f"  Regions:")
    print(f"    - Rotor core: {ROTOR_INNER_DIAMETER/2:.1f} to {rotor_core_outer:.1f} mm")
    print(f"    - Center air: 0 to {ROTOR_INNER_DIAMETER/2:.1f} mm")
    print(f"    - Inner air: {ROTOR_INNER_DIAMETER/2:.1f} to {airgap_middle_r/mm_to_m:.1f} mm (with rotor core and magnet cutouts)")
    print(f"    - Outer air: {airgap_middle_r/mm_to_m:.1f} to {outer_air_r/mm_to_m:.1f} mm")
    print(f"    - Shell: {outer_air_r/mm_to_m:.1f} to {domain_outer_r/mm_to_m:.1f} mm")
    print(f"  Magnetization: Radial, {NUM_POLES} poles (N={NUM_POLES//2}, S={NUM_POLES//2})")
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
        output_files = ['spm_motor_az.pos', 'spm_motor_b.pos', 'spm_motor_h.pos']
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
    
    mesh_file = os.path.join(work_dir, 'spm_motor.msh')
    b_field_file = os.path.join(work_dir, 'spm_motor_b.pos')
    
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
    shaft_r = ROTOR_INNER_DIAMETER / 2
    rotor_inner_r = ROTOR_INNER_DIAMETER / 2
    rotor_outer_r = ROTOR_OUTER_DIAMETER / 2
    magnet_thickness = MAGNET_THICKNESS
    magnet_outer_r = rotor_outer_r + magnet_thickness
    airgap = AIRGAP_LENGTH
    stator_inner_r = STATOR_INNER_DIAMETER / 2
    stator_outer_r = STATOR_OUTER_DIAMETER / 2
    
    # Calculate key radii
    slot_depth = SLOT_HEIGHT
    slot_bottom_r = stator_outer_r - slot_depth
    
    # Magnet arc parameters
    pole_pitch_angle = 2 * np.pi / NUM_POLES
    magnet_arc_angle = pole_pitch_angle * MAGNET_ARC_RATIO
    half_arc = magnet_arc_angle / 2
    
    # Slot parameters
    slot_angle = 2 * np.pi / NUM_SLOTS
    
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
    for i in range(NUM_POLES):
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
    for i in range(NUM_SLOTS):
        slot_center_angle = i * slot_angle
        
        # Draw rectangular slot with winding block
        # Parameters: angle, inner_radius, slot_height, slot_width, slot_opening, tooth_tip_height
        draw_rectangular_slot(ax, slot_center_angle, stator_inner_r, slot_depth, 
                             SLOT_WIDTH, SLOT_OPENING, 0.5)
    
    # Add title
    ax.set_title(f'{NUM_POLES}-Pole, {NUM_SLOTS}-Slot Permanent Magnet Motor\n' +
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
        f"Poles: {NUM_POLES}  |  Slots: {NUM_SLOTS}\n"
        f"Stack Length: {STACK_LENGTH:.1f}mm\n"
        f"\n"
        f"Rotor:\n"
        f"  ID: {ROTOR_INNER_DIAMETER:.1f}mm\n"
        f"  OD: {ROTOR_OUTER_DIAMETER:.1f}mm\n"
        f"\n"
        f"Magnets:\n"
        f"  Thickness: {magnet_thickness:.1f}mm\n"
        f"  Arc ratio: {MAGNET_ARC_RATIO:.2f}\n"
        f"  Hc: {abs(MAGNET_HC)/1000:.0f}kA/m\n"
        f"\n"
        f"Stator:\n"
        f"  ID: {STATOR_INNER_DIAMETER:.1f}mm\n"
        f"  OD: {STATOR_OUTER_DIAMETER:.1f}mm\n"
        f"  Slot depth: {SLOT_HEIGHT:.1f}mm\n"
        f"\n"
        f"Airgap: {airgap:.1f}mm"
    )
    ax.text(0.02, 0.98, param_text, transform=ax.transAxes,
            fontsize=8, verticalalignment='top', family='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    plt.tight_layout()
    
    # Save figure
    plot_file = os.path.join(work_dir, 'motor_geometry.png')
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"✓ Complete motor geometry plot saved: {os.path.basename(plot_file)}")
    print(f"  - Shows: Shaft, Rotor Core, {NUM_POLES} Magnets, Airgap, {NUM_SLOTS} Slots, Stator")
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
    
    b_pos_file = os.path.join(work_dir, 'spm_motor_b.pos')
    
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
    stator_inner_r = STATOR_INNER_DIAMETER / 2 * mm_to_m
    stator_outer_r = STATOR_OUTER_DIAMETER / 2 * mm_to_m
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
    rotor_outer_r = ROTOR_OUTER_DIAMETER / 2 * mm_to_m
    airgap = AIRGAP_LENGTH * mm_to_m
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


def parse_pos_file_with_vectors(pos_file):
    """Parse GetDP .pos file to extract vector field components.
    
    Returns:
        tuple: (points, triangles, bx_values, by_values, b_mag_values) where
            points: Nx2 array of (x,y) coordinates
            triangles: Mx3 array of triangle vertex indices  
            bx_values: N array of x-component at each point
            by_values: N array of y-component at each point
            b_mag_values: N array of field magnitudes
    """
    points = []
    triangles = []
    bx_values = []
    by_values = []
    b_mag_values = []
    
    with open(pos_file, 'r') as f:
        lines = f.readlines()
    
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        
        # Look for vector triangle elements
        if line.startswith('VT('):
            try:
                coords_part = line[3:line.index('){')]
                values_part = line[line.index('){')+2:line.index('};')]
                
                coords = [float(x) for x in coords_part.split(',')]
                vals = [float(x) for x in values_part.split(',')]
                
                # Extract x,y coordinates (ignore z)
                p1 = [coords[0], coords[1]]
                p2 = [coords[3], coords[4]]
                p3 = [coords[6], coords[7]]
                
                idx_start = len(points)
                points.extend([p1, p2, p3])
                triangles.append([idx_start, idx_start+1, idx_start+2])
                
                # Extract vector components (Bx, By, Bz at each vertex)
                for j in range(3):
                    bx = vals[j*3 + 0]
                    by = vals[j*3 + 1]
                    bz = vals[j*3 + 2]
                    mag = np.sqrt(bx**2 + by**2 + bz**2)
                    
                    bx_values.append(bx)
                    by_values.append(by)
                    b_mag_values.append(mag)
            except (ValueError, IndexError):
                pass
            
        i += 1
    
    return (np.array(points), np.array(triangles), 
            np.array(bx_values), np.array(by_values), np.array(b_mag_values))


def calculate_torque_maxwell(b_pos_file):
    """
    Calculate electromagnetic torque using Maxwell stress tensor method.
    
    Integrates Maxwell stress along airgap circle:
        T = r * L_stack * ∫(Br * Btheta / μ0) * r * dθ
    
    Args:
        b_pos_file: Path to B-field .pos file
        
    Returns:
        dict: {
            'torque_Nm': Total torque in N-m,
            'torque_density': Torque per rotor volume (kN-m/m³),
            'airgap_radius_m': Airgap radius in meters,
            'stress_angle_deg': Angular positions (degrees),
            'stress_distribution': Tangential stress at each angle (N/m²)
        }
    """
    print("\n" + "=" * 60)
    print("MAXWELL STRESS TENSOR TORQUE CALCULATION")
    print("=" * 60)
    
    # Parse B-field data
    points, triangles, bx, by, b_mag = parse_pos_file_with_vectors(b_pos_file)
    
    # Convert dimensions
    mm_to_m = 0.001
    rotor_outer_r = ROTOR_OUTER_DIAMETER / 2 * mm_to_m
    airgap = AIRGAP_LENGTH * mm_to_m
    airgap_middle_r = rotor_outer_r + airgap / 2
    stack_length = STACK_LENGTH * mm_to_m
    
    # Extract points near airgap middle radius
    r_points = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
    tolerance = 0.5 * mm_to_m  # 0.5mm tolerance
    airgap_mask = np.abs(r_points - airgap_middle_r) < tolerance
    
    airgap_points = points[airgap_mask]
    airgap_bx = bx[airgap_mask]
    airgap_by = by[airgap_mask]
    
    # Calculate angular position
    theta = np.arctan2(airgap_points[:, 1], airgap_points[:, 0])
    
    # Convert to radial and tangential components
    # Br = Bx*cos(theta) + By*sin(theta)
    # Btheta = -Bx*sin(theta) + By*cos(theta)
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    
    br = airgap_bx * cos_theta + airgap_by * sin_theta
    btheta = -airgap_bx * sin_theta + airgap_by * cos_theta
    
    # Maxwell stress: sigma_theta = Br * Btheta / μ0
    mu0 = 4 * np.pi * 1e-7  # H/m
    stress = br * btheta / mu0
    
    # Sort by angle for integration
    sort_idx = np.argsort(theta)
    theta_sorted = theta[sort_idx]
    stress_sorted = stress[sort_idx]
    
    # Integrate using trapezoidal rule: T = r * L * ∫(stress * r) dθ
    from scipy import integrate
    torque = airgap_middle_r * stack_length * integrate.trapezoid(stress_sorted * airgap_middle_r, theta_sorted)
    
    # Calculate torque density
    rotor_volume = np.pi * (rotor_outer_r**2 - (ROTOR_INNER_DIAMETER/2*mm_to_m)**2) * stack_length
    torque_density_kNm_m3 = (torque / rotor_volume) / 1000  # kN-m/m³
    
    print(f"\n  Airgap radius: {airgap_middle_r*1000:.2f} mm")
    print(f"  Stack length: {stack_length*1000:.1f} mm")
    print(f"  Integration points: {len(theta_sorted)}")
    print(f"  Br range: {br.min():.4f} to {br.max():.4f} T")
    print(f"  Bθ range: {btheta.min():.4f} to {btheta.max():.4f} T")
    print(f"  Stress range: {stress.min()/1000:.2f} to {stress.max()/1000:.2f} kPa")
    print(f"\n  ✓ TOTAL TORQUE: {torque:.4f} N-m")
    print(f"  ✓ Torque Density: {torque_density_kNm_m3:.2f} kN-m/m³")
    
    return {
        'torque_Nm': torque,
        'torque_density': torque_density_kNm_m3,
        'airgap_radius_m': airgap_middle_r,
        'stress_angle_deg': np.degrees(theta_sorted),
        'stress_distribution': stress_sorted
    }


def calculate_flux_linkage(b_pos_file, phase='A'):
    """
    Calculate flux linkage for one phase winding using airgap flux integration.
    
    Method: Integrate radial flux density around airgap, then apply winding distribution.
    This is the industry-standard approach and matches physical measurements.
    
    For distributed 3-phase winding:
    1. Calculate flux per pole: Φ_pole = ∫ Br(θ) × r × dθ × L_stack
    2. Apply winding distribution factor for phase
    3. λ_phase = N_series × Φ_linked
    
    Args:
        b_pos_file: Path to B-field .pos file (not Az!)
        phase: 'A', 'B', or 'C'
        
    Returns:
        dict: {
            'flux_linkage_Wb': Total flux linkage in Weber-turns,
            'flux_per_turn_Wb': Flux linkage per series turn,
            'flux_per_pole': Flux per pole (Wb),
            'num_slots': Number of slots for this phase,
            'num_series_turns': Total series turns for phase
        }
    """
    # Parse B-field data
    points, triangles, bx, by, b_mag = parse_pos_file_with_vectors(b_pos_file)
    
    # Get winding assignment
    winding_phases = assign_winding_phases(NUM_SLOTS, NUM_POLES)
    
    # Convert dimensions
    mm_to_m = 0.001
    rotor_outer_r = ROTOR_OUTER_DIAMETER / 2 * mm_to_m
    airgap = AIRGAP_LENGTH * mm_to_m
    airgap_middle_r = rotor_outer_r + airgap / 2
    stack_length = STACK_LENGTH * mm_to_m
    
    # Extract points near airgap middle radius
    r_points = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
    tolerance = 0.5 * mm_to_m  # 0.5mm tolerance
    airgap_mask = np.abs(r_points - airgap_middle_r) < tolerance
    
    airgap_points = points[airgap_mask]
    airgap_bx = bx[airgap_mask]
    airgap_by = by[airgap_mask]
    
    # Calculate angular position and radial B-component
    theta = np.arctan2(airgap_points[:, 1], airgap_points[:, 0])
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    
    # Radial component: Br = Bx*cos(θ) + By*sin(θ)
    br = airgap_bx * cos_theta + airgap_by * sin_theta
    
    # Sort by angle for integration
    sort_idx = np.argsort(theta)
    theta_sorted = theta[sort_idx]
    br_sorted = br[sort_idx]
    
    # Calculate flux per pole by integrating over one pole pitch
    num_poles = NUM_POLES
    pole_pitch = 2 * np.pi / num_poles
    
    # For each pole, integrate Br over that pole span
    # Flux: Φ = ∫ Br × r × dθ × L_stack
    from scipy import integrate
    
    # Method: Integrate over full 360° then divide by number of poles for average
    total_flux = integrate.trapezoid(br_sorted * airgap_middle_r, theta_sorted) * stack_length
    flux_per_pole = abs(total_flux / num_poles)  # Average flux per pole
    
    # Now calculate winding factor and linked flux for this phase
    # For distributed winding with q slots per pole per phase:
    num_slots = NUM_SLOTS
    slots_per_pole_per_phase = num_slots / (num_poles * 3)  # q = 48/(8*3) = 2
    
    # Get phase slot indices and their electrical angles
    phase_slots = []
    for slot_idx in range(num_slots):
        slot_phase, polarity = winding_phases[slot_idx]
        if slot_phase == phase:
            # Electrical angle of this slot (relative to phase A reference)
            slot_angle_mech = slot_idx * 2 * np.pi / num_slots
            slot_angle_elec = slot_angle_mech * (num_poles / 2)  # Electrical angle
            phase_slots.append((slot_idx, polarity, slot_angle_elec))
    
    # Calculate winding factor using phasor sum
    # Each coil links flux with magnitude proportional to its position
    # Winding factor: kw = (sum of phasors) / (arithmetic sum)
    phasor_sum_real = 0.0
    phasor_sum_imag = 0.0
    for slot_idx, polarity, elec_angle in phase_slots:
        # Each slot contributes a phasor at its electrical angle
        phasor_sum_real += polarity * np.cos(elec_angle)
        phasor_sum_imag += polarity * np.sin(elec_angle)
    
    phasor_magnitude = np.sqrt(phasor_sum_real**2 + phasor_sum_imag**2)
    num_phase_slots = len(phase_slots)
    
    # Winding factor
    kw = phasor_magnitude / num_phase_slots if num_phase_slots > 0 else 0
    
    # Effective flux linked by this phase
    # Each pole contributes, weighted by winding distribution
    num_pole_pairs = num_poles // 2
    flux_linked = flux_per_pole * kw * num_pole_pairs  # Fundamental component
    
    # Total flux linkage with series turns
    num_series_turns = TURNS_PER_COIL * num_phase_slots
    flux_linkage = flux_linked * num_series_turns
    
    # Per-turn linkage
    flux_per_turn = flux_linkage / num_series_turns if num_series_turns > 0 else 0
    
    return {
        'flux_linkage_Wb': flux_linkage,
        'flux_per_turn_Wb': flux_per_turn,
        'flux_per_pole': flux_per_pole,
        'winding_factor': kw,
        'num_slots': num_phase_slots,
        'num_series_turns': num_series_turns
    }


def calculate_all_phases_linkage(b_pos_file):
    """Calculate flux linkage for all three phases using airgap flux method."""
    results = {}
    
    print("\n" + "=" * 60)
    print("FLUX LINKAGE CALCULATION (Airgap Flux Method)")
    print("=" * 60)
    
    for phase in ['A', 'B', 'C']:
        result = calculate_flux_linkage(b_pos_file, phase)
        results[phase] = result
        
        print(f"\n  Phase {phase}:")
        print(f"    Slots: {result['num_slots']}")
        print(f"    Series turns: {result['num_series_turns']}")
        print(f"    Winding factor (kw): {result['winding_factor']:.4f}")
        print(f"    Flux per pole: {result['flux_per_pole']*1000:.4f} mWb")
        print(f"    Flux linkage: {result['flux_linkage_Wb']:.6f} Wb-turns")
        print(f"    Flux per turn: {result['flux_per_turn_Wb']*1000:.4f} mWb")
    
    # Calculate average
    avg_linkage = np.mean([results[p]['flux_linkage_Wb'] for p in ['A', 'B', 'C']])
    avg_kw = np.mean([results[p]['winding_factor'] for p in ['A', 'B', 'C']])
    print(f"\n  Average flux linkage: {avg_linkage:.6f} Wb-turns")
    print(f"  Average winding factor: {avg_kw:.4f}")
    
    return results


def display_motor_constants(torque_result, linkage_results):
    """
    Display calculated motor constants and performance metrics.
    
    Args:
        torque_result: Dictionary from calculate_torque_maxwell
        linkage_results: Dictionary from calculate_all_phases_linkage
    """
    print("\n" + "━" * 60)
    print("MOTOR CONSTANTS & PERFORMANCE METRICS")
    print("━" * 60)
    
    torque_Nm = torque_result['torque_Nm']
    avg_linkage = np.mean([linkage_results[p]['flux_linkage_Wb'] for p in ['A', 'B', 'C']])
    
    # Get current density and calculate RMS phase current
    mm_to_m = 0.001
    slot_depth = SLOT_HEIGHT * mm_to_m
    slot_width = SLOT_WIDTH * mm_to_m
    slot_area = slot_depth * slot_width
    
    # Current density (set by user)
    J_rms = WINDING_CURRENT_DENSITY  # A/m²
    
    # RMS current per slot = J * slot_area
    I_slot_rms = J_rms * slot_area
    
    # Phase RMS current (all phase slots in parallel or series depending on winding)
    # For distributed winding, typically series connection
    I_phase_rms = I_slot_rms  # Each slot carries same current in series winding
    
    # Torque constant: Kt = Torque / I_phase_rms
    if I_phase_rms > 0:
        Kt = torque_Nm / I_phase_rms  # N-m / A
    else:
        Kt = 0
    
    # Back-EMF constant: Ke = λ / (for rotation)
    # At 1000 RPM: ω = 1000 * 2π/60 = 104.72 rad/s
    # Back-EMF = ω * λ
    # Ke in V/krpm: Ke = λ * (1000*2π/60) / 1000 = λ * 0.10472
    Ke_V_krpm = avg_linkage * 1000 * 2 * np.pi / 60  # Line-to-neutral
    Ke_V_krpm_LL = Ke_V_krpm * np.sqrt(3)  # Line-to-line for 3-phase
    
    # Power at base speed (if defined)
    # Assuming base speed around 3000-5000 RPM for typical motor
    base_speed_rpm = 3000
    omega_base = base_speed_rpm * 2 * np.pi / 60
    power_base_kW = torque_Nm * omega_base / 1000
    
    print(f"\n{'Electromagnetic Torque:':<30} {torque_Nm:>10.4f} N-m")
    print(f"{'Torque Density:':<30} {torque_result['torque_density']:>10.2f} kN-m/m³")
    
    print(f"\n{'Phase Current (RMS):':<30} {I_phase_rms:>10.2f} A")
    print(f"{'Current Density:':<30} {J_rms/1e6:>10.2f} A/mm²")
    
    if I_phase_rms > 0:
        print(f"\n{'Torque Constant (Kt):':<30} {Kt:>10.4f} N-m/A_rms")
    
    print(f"\n{'Average Flux Linkage:':<30} {avg_linkage:>10.6f} Wb-turns")
    print(f"{'Back-EMF Constant (Ke):':<30} {Ke_V_krpm:>10.3f} V/krpm (L-N)")
    print(f"{'Back-EMF Constant (Ke):':<30} {Ke_V_krpm_LL:>10.3f} V/krpm (L-L)")
    
    print(f"\n{'Power @ {base_speed_rpm} RPM:':<30} {power_base_kW:>10.2f} kW")
    
    print("━" * 60)
    
    return {
        'Kt': Kt,
        'Ke_V_krpm_LN': Ke_V_krpm,
        'Ke_V_krpm_LL': Ke_V_krpm_LL,
        'I_phase_rms': I_phase_rms,
        'power_base_kW': power_base_kW
    }


def plot_torque_and_linkage(torque_result, linkage_results):
    """
    Create comprehensive visualization of torque and flux linkage results.
    
    Args:
        torque_result: Dictionary from calculate_torque_maxwell
        linkage_results: Dictionary from calculate_all_phases_linkage
    """
    print("\n" + "=" * 60)
    print("Creating Torque & Flux Linkage Plots")
    print("=" * 60)
    
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
    
    # --- Subplot 1: Maxwell Stress Distribution ---
    ax1 = fig.add_subplot(gs[0, :])
    
    theta_deg = torque_result['stress_angle_deg']
    stress_kPa = torque_result['stress_distribution'] / 1000  # Convert to kPa
    
    ax1.plot(theta_deg, stress_kPa, 'b-', linewidth=2, label='Tangential Stress')
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax1.fill_between(theta_deg, 0, stress_kPa, alpha=0.3)
    
    ax1.set_xlabel('Angular Position (degrees)', fontsize=12)
    ax1.set_ylabel('Maxwell Stress σθ (kPa)', fontsize=12)
    ax1.set_title(f'Maxwell Stress Tensor Distribution (Airgap r={torque_result["airgap_radius_m"]*1000:.1f}mm)', 
                  fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=10)
    ax1.set_xlim([-180, 180])
    
    # Add torque annotation
    torque_text = f'Total Torque: {torque_result["torque_Nm"]:.4f} N-m'
    ax1.text(0.98, 0.97, torque_text, transform=ax1.transAxes,
             fontsize=12, fontweight='bold', va='top', ha='right',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # --- Subplot 2: Flux Linkage per Phase ---
    ax2 = fig.add_subplot(gs[1, 0])
    
    phases = ['A', 'B', 'C']
    linkages = [linkage_results[p]['flux_linkage_Wb'] for p in phases]
    colors = ['red', 'green', 'blue']
    
    bars = ax2.bar(phases, linkages, color=colors, alpha=0.7, edgecolor='black', linewidth=1.5)
    
    # Add value labels on bars
    for bar, linkage in zip(bars, linkages):
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height,
                f'{linkage:.6f}\nWb-turns',
                ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    ax2.set_ylabel('Flux Linkage (Wb-turns)', fontsize=12)
    ax2.set_title('Flux Linkage per Phase', fontsize=14, fontweight='bold')
    ax2.grid(True, axis='y', alpha=0.3)
    ax2.set_ylim([0, max(linkages) * 1.15])
    
    # --- Subplot 3: Winding Factor Visualization (Phase positions) ---
    ax3 = fig.add_subplot(gs[1, 1], projection='polar')
    
    # Get winding assignment for visualization
    winding_phases = assign_winding_phases(NUM_SLOTS, NUM_POLES)
    
    # Plot slot positions for each phase
    colors_phase = {'A': 'red', 'B': 'green', 'C': 'blue'}
    for phase in ['A', 'B', 'C']:
        phase_angles = []
        phase_polarities = []
        for slot_idx in range(NUM_SLOTS):
            slot_phase, polarity = winding_phases[slot_idx]
            if slot_phase == phase:
                angle = slot_idx * 2 * np.pi / NUM_SLOTS
                phase_angles.append(angle)
                phase_polarities.append(polarity)
        
        # Plot positive and negative separately
        pos_angles = [a for a, p in zip(phase_angles, phase_polarities) if p > 0]
        neg_angles = [a for a, p in zip(phase_angles, phase_polarities) if p < 0]
        
        ax3.scatter(pos_angles, [1]*len(pos_angles), c=colors_phase[phase], 
                   marker='o', s=100, alpha=0.7, edgecolors='black', linewidth=1.5,
                   label=f'Phase {phase} (+)')
        ax3.scatter(neg_angles, [1]*len(neg_angles), c=colors_phase[phase], 
                   marker='x', s=100, alpha=0.7, linewidth=2,
                   label=f'Phase {phase} (-)')
    
    ax3.set_ylim(0, 1.2)
    ax3.set_title(f'Winding Distribution\n({NUM_SLOTS} slots, {NUM_POLES} poles)', 
                  fontsize=14, fontweight='bold', pad=20)
    ax3.legend(loc='upper left', bbox_to_anchor=(1.1, 1.0), fontsize=8)
    ax3.set_yticks([])
    
    # --- Subplot 4: Series Turns per Phase ---
    ax4 = fig.add_subplot(gs[2, 0])
    
    series_turns = [linkage_results[p]['num_series_turns'] for p in phases]
    num_slots_phase = [linkage_results[p]['num_slots'] for p in phases]
    
    x = np.arange(len(phases))
    width = 0.35
    
    bars1 = ax4.bar(x - width/2, num_slots_phase, width, label='Slots per Phase', 
                    color='lightblue', edgecolor='black', linewidth=1)
    bars2 = ax4.bar(x + width/2, series_turns, width, label='Series Turns',
                    color='orange', edgecolor='black', linewidth=1)
    
    ax4.set_ylabel('Count', fontsize=12)
    ax4.set_title('Winding Configuration', fontsize=14, fontweight='bold')
    ax4.set_xticks(x)
    ax4.set_xticklabels(phases)
    ax4.legend(fontsize=10)
    ax4.grid(True, axis='y', alpha=0.3)
    
    # Add value labels
    for bars in [bars1, bars2]:
        for bar in bars:
            height = bar.get_height()
            ax4.text(bar.get_x() + bar.get_width()/2., height,
                    f'{int(height)}', ha='center', va='bottom', fontsize=9)
    
    # --- Subplot 5: Motor Constants Summary ---
    ax5 = fig.add_subplot(gs[2, 1])
    ax5.axis('off')
    
    # Calculate motor constants for display
    avg_linkage = np.mean(linkages)
    torque_Nm = torque_result['torque_Nm']
    
    mm_to_m = 0.001
    slot_area = SLOT_HEIGHT * SLOT_WIDTH * mm_to_m * mm_to_m
    I_phase_rms = WINDING_CURRENT_DENSITY * slot_area
    
    if I_phase_rms > 0:
        Kt = torque_Nm / I_phase_rms
    else:
        Kt = 0
    
    Ke_V_krpm_LL = avg_linkage * 1000 * 2 * np.pi / 60 * np.sqrt(3)
    
    summary_text = f"""
    MOTOR CONSTANTS SUMMARY
    ═══════════════════════════════
    
    Torque:              {torque_Nm:.4f} N-m
    Torque Density:      {torque_result['torque_density']:.2f} kN-m/m³
    
    Avg Flux Linkage:    {avg_linkage:.6f} Wb-turns
    
    Phase Current:       {I_phase_rms:.2f} A (RMS)
    Current Density:     {WINDING_CURRENT_DENSITY/1e6:.2f} A/mm²
    
    Kt (Torque Const):   {Kt:.4f} N-m/A
    Ke (Back-EMF):       {Ke_V_krpm_LL:.3f} V/krpm (L-L)
    
    Poles / Slots:       {NUM_POLES} / {NUM_SLOTS}
    Stack Length:        {STACK_LENGTH:.1f} mm
    Airgap Length:       {AIRGAP_LENGTH:.2f} mm
    """
    
    ax5.text(0.1, 0.9, summary_text, transform=ax5.transAxes,
             fontsize=11, fontfamily='monospace', va='top',
             bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
    
    # Overall title
    fig.suptitle(f'{NUM_POLES}-Pole, {NUM_SLOTS}-Slot Motor - Electromagnetic Performance Analysis',
                 fontsize=16, fontweight='bold', y=0.995)
    
    # Save figure
    plot_file = os.path.join(work_dir, 'torque_flux_linkage_analysis.png')
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"\n✓ Torque & flux linkage plot saved: {os.path.basename(plot_file)}")
    
    # Show non-blocking
    plt.show(block=False)
    plt.pause(0.1)
    
    return True


def run_rotation_study():
    """
    Run FEA at multiple rotor positions to compute torque waveform.
    
    This function performs a parametric sweep of rotor mechanical angle positions,
    computing torque at each position to capture cogging torque and total electromagnetic
    torque profiles. Useful for analyzing torque ripple and optimizing design.
    
    Returns:
        dict: {
            'angles_deg': array of mechanical angles,
            'torques_Nm': array of torque values,
            'angles_elec_deg': array of electrical angles,
            'num_positions': number of positions analyzed
        }
    """
    print("\n" + "=" * 60)
    print("ROTOR ROTATION STUDY")
    print("=" * 60)
    print(f"\nRotation range: {ROTATION_START_DEG}° to {ROTATION_END_DEG}° mechanical")
    print(f"Number of positions: {ROTATION_STEPS}")
    print(f"Analysis type: {'PM + Winding' if WINDING_CURRENT_DENSITY > 0 else 'PM Only (Cogging)'}")
    
    # Calculate rotor positions
    angles_mech = np.linspace(ROTATION_START_DEG, ROTATION_END_DEG, ROTATION_STEPS)
    torques = []
    
    # Electrical angle conversion (for 8 poles: 1 mech cycle = 4 elec cycles)
    num_pole_pairs = NUM_POLES / 2
    angles_elec = angles_mech * num_pole_pairs
    
    print(f"\nElectrical angle range: {angles_elec[0]:.1f}° to {angles_elec[-1]:.1f}°")
    print(f"({angles_elec[-1] - angles_elec[0]:.1f}° = {(angles_elec[-1] - angles_elec[0])/360:.2f} electrical cycles)")
    print(f"Base torque angle: {WINDING_CURRENT_ANGLE}° electrical")
    print(f"  → Current angle advances with rotor rotation")
    print(f"  → Maintains constant torque angle (MMF-to-rotor relationship)")
    print(f"  → Torque ripple from slot effects and winding harmonics")
    
    # Run FEA at each position
    for i, angle in enumerate(angles_mech):
        # Advance current angle electrically with rotor to maintain constant torque angle
        # WINDING_CURRENT_ANGLE = base torque angle (e.g., 0° for d-axis, 90° for q-axis)
        # angles_elec[i] = electrical rotation of rotor
        # Sum keeps MMF-to-rotor relationship constant
        current_angle = WINDING_CURRENT_ANGLE + angles_elec[i]
        
        print(f"\n{'─'*60}")
        print(f"Position {i+1}/{ROTATION_STEPS}: θ_mech = {angle:.2f}° (θ_elec = {angles_elec[i]:.2f}°)")
        print(f"  Winding current angle: {current_angle:.2f}° electrical")
        print(f"  (Base: {WINDING_CURRENT_ANGLE:.2f}° + Rotor elec: {angles_elec[i]:.2f}°)")
        print(f"{'─'*60}")
        
        # Step 1: Create geometry with rotor at this position
        geo_file = create_geometry(rotor_angle_deg=angle)
        if not geo_file:
            print(f"❌ Failed to create geometry at angle {angle}°")
            continue
        
        # Step 2: Generate mesh
        msh_file = generate_mesh(geo_file)
        if not msh_file:
            print(f"❌ Failed to generate mesh at angle {angle}°")
            continue
        
        # Step 3: Create solver file with rotor angle for Hc vector rotation
        pro_file = create_solver_file(MAGNET_HC, WINDING_CURRENT_DENSITY, current_angle, USE_NONLINEAR_STEEL, rotor_angle_deg=angle)
        if not pro_file:
            print(f"❌ Failed to create solver file at angle {angle}°")
            continue
        
        # Step 4: Run solver
        success = run_solver(pro_file, msh_file)
        if not success:
            print(f"❌ Solver failed at angle {angle}°")
            continue
        
        # Step 5: Calculate torque
        b_pos_file = os.path.join(work_dir, 'spm_motor_b.pos')
        if os.path.exists(b_pos_file):
            torque_result = calculate_torque_maxwell(b_pos_file)
            torques.append(torque_result['torque_Nm'])
            print(f"✓ Torque: {torque_result['torque_Nm']:.4f} N-m")
        else:
            print(f"❌ B-field file not found at angle {angle}°")
            torques.append(0.0)
    
    print(f"\n{'='*60}")
    print(f"ROTATION STUDY COMPLETE")
    print(f"{'='*60}")
    
    torques_array = np.array(torques)
    print(f"\nBase torque angle: {WINDING_CURRENT_ANGLE}° (current synchronized with rotor)")
    print(f"\nTorque Statistics:")
    print(f"  Average:  {torques_array.mean():.4f} N-m")
    print(f"  Peak:     {torques_array.max():.4f} N-m")
    print(f"  Valley:   {torques_array.min():.4f} N-m")
    print(f"  Ripple:   {torques_array.max() - torques_array.min():.4f} N-m")
    print(f"  Ripple %: {100 * (torques_array.max() - torques_array.min()) / torques_array.mean():.2f}%")
    
    return {
        'angles_deg': angles_mech,
        'torques_Nm': torques_array,
        'angles_elec_deg': angles_elec,
        'num_positions': ROTATION_STEPS
    }


def plot_torque_waveform(rotation_results):
    """
    Plot torque waveform from rotation study.
    
    Creates comprehensive visualization showing:
    - Torque vs mechanical angle
    - Torque vs electrical angle
    - FFT analysis for harmonic content
    - Statistics panel
    
    Args:
        rotation_results: dict from run_rotation_study()
    """
    print("\n" + "=" * 60)
    print("Creating Torque Waveform Visualization")
    print("=" * 60)
    
    angles_mech = rotation_results['angles_deg']
    angles_elec = rotation_results['angles_elec_deg']
    torques = rotation_results['torques_Nm']
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
    
    # Subplot 1: Torque vs Mechanical Angle
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(angles_mech, torques, 'b-', linewidth=2, marker='o', markersize=4)
    ax1.axhline(y=torques.mean(), color='r', linestyle='--', linewidth=1, label=f'Average: {torques.mean():.4f} N-m')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlabel('Mechanical Angle (degrees)', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Torque (N-m)', fontsize=11, fontweight='bold')
    ax1.set_title('Torque vs Rotor Position (Mechanical Angle)', fontsize=13, fontweight='bold')
    ax1.legend()
    
    # Subplot 2: Torque vs Electrical Angle
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(angles_elec, torques, 'g-', linewidth=2, marker='s', markersize=4)
    ax2.axhline(y=torques.mean(), color='r', linestyle='--', linewidth=1)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlabel('Electrical Angle (degrees)', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Torque (N-m)', fontsize=11, fontweight='bold')
    ax2.set_title('Torque vs Electrical Angle', fontsize=12, fontweight='bold')
    
    # Subplot 3: FFT Analysis
    ax3 = fig.add_subplot(gs[1, 1])
    # Perform FFT
    from scipy import signal
    fft_vals = np.fft.rfft(torques - torques.mean())
    fft_freqs = np.fft.rfftfreq(len(torques), d=(angles_elec[1] - angles_elec[0]) / 360)  # cycles per electrical revolution
    fft_magnitude = np.abs(fft_vals) * 2 / len(torques)
    
    # Plot harmonics
    ax3.stem(fft_freqs[:20], fft_magnitude[:20], basefmt=' ')
    ax3.grid(True, alpha=0.3)
    ax3.set_xlabel('Harmonic Order (per elec. cycle)', fontsize=11, fontweight='bold')
    ax3.set_ylabel('Magnitude (N-m)', fontsize=11, fontweight='bold')
    ax3.set_title('Torque Harmonic Content (FFT)', fontsize=12, fontweight='bold')
    
    # Subplot 4: Statistics Panel
    ax4 = fig.add_subplot(gs[2, :])
    ax4.axis('off')
    
    # Calculate statistics
    torque_mean = torques.mean()
    torque_max = torques.max()
    torque_min = torques.min()
    torque_ripple = torque_max - torque_min
    torque_ripple_pct = 100 * torque_ripple / torque_mean
    
    # Find dominant harmonics
    harmonic_indices = np.argsort(fft_magnitude)[-5:][::-1]  # Top 5
    
    # Determine analysis description
    if WINDING_CURRENT_DENSITY > 0:
        analysis_desc = f'Electromagnetic (Torque angle: {WINDING_CURRENT_ANGLE}°, synchronized)'
    else:
        analysis_desc = 'Cogging (PM Only)'
    
    stats_text = f"""
    TORQUE WAVEFORM ANALYSIS SUMMARY
    {'='*80}
    
    Analysis Configuration:
      Mechanical angle range:    {angles_mech[0]:.2f}° to {angles_mech[-1]:.2f}° ({angles_mech[-1] - angles_mech[0]:.2f}° span)
      Electrical angle range:    {angles_elec[0]:.2f}° to {angles_elec[-1]:.2f}° ({(angles_elec[-1] - angles_elec[0])/360:.2f} cycles)
      Number of positions:       {len(torques)}
      Number of poles:           {NUM_POLES} ({NUM_POLES/2:.0f} pole pairs)
      Base torque angle:         {WINDING_CURRENT_ANGLE}° electrical (synchronized with rotor)
      Analysis type:             {analysis_desc}
      
    Torque Statistics:
      Average torque:            {torque_mean:.4f} N-m
      Peak torque:               {torque_max:.4f} N-m
      Minimum torque:            {torque_min:.4f} N-m
      Peak-to-peak ripple:       {torque_ripple:.4f} N-m  ({torque_ripple_pct:.2f}% of average)
      Torque density:            {torque_mean / (np.pi * (ROTOR_OUTER_DIAMETER/2000)**2 * STACK_LENGTH/1000):.2f} kN-m/m³
      
    Dominant Harmonics (Top 5):
    """
    
    for idx in harmonic_indices[:5]:
        if fft_freqs[idx] > 0:  # Skip DC component
            stats_text += f"      Harmonic {fft_freqs[idx]:.1f}:  {fft_magnitude[idx]:.4f} N-m\n"
    
    ax4.text(0.05, 0.95, stats_text, transform=ax4.transAxes,
             fontsize=10, verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
    
    # Overall title
    if WINDING_CURRENT_DENSITY > 0:
        analysis_type = f'Electromagnetic Torque (Torque angle: {WINDING_CURRENT_ANGLE}°, synchronized)'
    else:
        analysis_type = 'Cogging Torque'
    fig.suptitle(f'{NUM_POLES}-Pole Motor: {analysis_type} Waveform Analysis',
                 fontsize=16, fontweight='bold', y=0.98)
    
    # Save figure
    plot_file = os.path.join(work_dir, 'torque_waveform_analysis.png')
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"\n✓ Torque waveform plot saved: {os.path.basename(plot_file)}")
    
    # Show
    plt.show(block=False)
    plt.pause(0.1)
    
    return True


def main():
    """
    Main execution - Complete motor FEA workflow with electromagnetic performance analysis
    
    Workflow:
    1. Load motor parameters from Excel (automatic)
    2. Create geometry (.geo) - 8 magnets, 48 slots, 5 air regions
    3. Generate mesh (.msh) - fine mesh at airgap and slots
    4. Create solver file (.pro) - PM + 3-phase winding excitation
    5. Run GetDP solver - magnetostatic FEA solution
    6. Visualize in Gmsh - mesh and field distribution
    7. Create motor geometry plot - complete cross-section
    8. Create flux density plots - 2D colormap and airgap profile
    9. Calculate electromagnetic torque - Maxwell stress tensor integration
    10. Calculate flux linkage - all three phases
    11. Display motor constants - Kt, Ke, performance metrics
    12. Create torque & linkage plots - comprehensive visualization
    
    All motor dimensions loaded dynamically from:
    projects/example_design_1/example_design_1_edrive_parameters.xlsx
    """
    
    print("\n")
    print("*" * 60)
    print(f"{NUM_POLES}-Pole Arc Magnet Rotor - Motor Parameters")
    print("Geometry -> Mesh -> Solve -> Visualize -> Analyze")
    print("*" * 60)
    print("\n")
    
    # Check if rotation study is enabled
    if ENABLE_ROTATION_STUDY:
        print("\n[ROTATION STUDY MODE ENABLED]")
        print(f"   Will analyze torque at {ROTATION_STEPS} rotor positions")
        print(f"   Range: {ROTATION_START_DEG}° to {ROTATION_END_DEG}° mechanical\n")
        
        # Run rotation study
        rotation_results = run_rotation_study()
        
        # Plot results
        plot_torque_waveform(rotation_results)
        
        # Keep plots open
        print("\nRotation study complete. Close plot windows to exit.")
        plt.show()  # Blocking call to keep all plots open
        
        print("\n" + "*" * 60)
        print("ROTATION STUDY FINISHED SUCCESSFULLY!")
        print("*" * 60)
        print()
        print(f"All files saved to: {work_dir}")
        print()
        
        return 0
    
    # Standard single-position analysis
    print("\n📍 SINGLE POSITION ANALYSIS MODE")
    print(f"   Rotor angle: 0.0° mechanical")
    print(f"   Winding current angle: {WINDING_CURRENT_ANGLE}° electrical\n")
    
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
    
    # Step 3: Create solver file (rotor at 0° reference position)
    pro_file = create_solver_file(MAGNET_HC, WINDING_CURRENT_DENSITY, WINDING_CURRENT_ANGLE, USE_NONLINEAR_STEEL, rotor_angle_deg=0.0)
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
        print("\n[!] Warning: Flux density plot generation failed, but solver completed successfully")
    
    # Step 7: Calculate electromagnetic torque (Maxwell stress tensor)
    b_pos_file = os.path.join(work_dir, 'spm_motor_b.pos')
    if os.path.exists(b_pos_file) and (MAGNET_HC > 0 or WINDING_CURRENT_DENSITY > 0):
        torque_result = calculate_torque_maxwell(b_pos_file)
    else:
        print("\n⚠ Skipping torque calculation (no excitation or B-field file not found)")
        torque_result = None
    
    # Step 8: Calculate flux linkage for all phases (using B-field, not Az)
    # Reuse the same B-field file from torque calculation
    if os.path.exists(b_pos_file) and (MAGNET_HC > 0 or WINDING_CURRENT_DENSITY > 0):
        linkage_results = calculate_all_phases_linkage(b_pos_file)
    else:
        print("\n⚠ Skipping flux linkage calculation (no excitation or B-field file not found)")
        linkage_results = None
    
    # Step 9: Display motor constants and performance metrics
    if torque_result and linkage_results:
        motor_constants = display_motor_constants(torque_result, linkage_results)
        
        # Step 10: Create torque and flux linkage visualization
        plot_torque_and_linkage(torque_result, linkage_results)
    else:
        print("\n⚠ Skipping motor constants calculation (torque or linkage data not available)")
    
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
    print("1. ✓ Geometry created (.geo file) - 8 arc magnets, 48 slots, 5 air regions")
    print("2. ✓ Mesh generated (.msh file) - fine airgap and slot mesh")
    print("3. ✓ Solver input created (.pro file) - PM + 3-phase winding excitation")
    print("4. ✓ FEA solution computed - magnetostatic field analysis")
    print("5. ✓ Results visualized in Gmsh - complete field distribution")
    print("6a. ✓ Motor geometry plot - complete cross-section visualization")
    print("6b. ✓ Flux density analysis - spatial and airgap profiles")
    print("7. ✓ Electromagnetic torque - Maxwell stress tensor integration")
    print("8. ✓ Flux linkage calculation - all three phases")
    print("9. ✓ Motor constants computed - Kt, Ke, performance metrics")
    print("10. ✓ Torque & linkage visualization - comprehensive performance plots")
    print()
    print("Complete electromagnetic performance analysis with torque and back-EMF!")
    print()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())


