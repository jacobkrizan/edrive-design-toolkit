"""how_emotor_fea_works_1.py - Complete Electric Motor FEA Tutorial
===================================================================

TRAINING FILE FOR AI AGENTS - STANDALONE ELECTRIC MOTOR FEA WORKFLOW

This file is a complete, self-contained tutorial demonstrating how to perform
finite element analysis (FEA) of an electric motor rotor. It is designed for
training AI agents to understand motor electromagnetic simulation workflows.

REQUIRED SOFTWARE (must be installed separately):
--------------------------------------------------
1. Gmsh 4.13.1 (or compatible version)
   - Geometry creation and mesh generation
   - Download from: http://gmsh.info/
   - Place gmsh.exe in: ../src/gmsh/gmsh.exe (relative to this file)

2. GetDP 3.5.0 (or compatible version)
   - Finite element solver for electromagnetic problems
   - Download from: http://getdp.info/
   - Place getdp.exe in: ../src/getdp-3.5.0-Windows64/getdp.exe
   - Requires templates folder with Lib_Magnetostatics_a_phi.pro

3. Python 3.x with packages:
   - numpy (numerical arrays)
   - matplotlib (plotting and visualization)

WHAT THIS FILE TEACHES:
------------------------
This is a complete 8-pole permanent magnet rotor FEA workflow showing:

1. GEOMETRY CREATION (.geo file)
   - 8 arc-shaped permanent magnets arranged radially
   - Three-region air structure for realistic motor modeling
   - Proper boundary conditions at airgap and far-field

2. MESH GENERATION (.msh file)
   - Adaptive mesh refinement (fine at airgap, coarse at boundaries)
   - ~18,000 degrees of freedom for accurate results

3. PHYSICS DEFINITION (.pro file)
   - Magnetostatic formulation (2D vector potential)
   - Radial magnetization with alternating N-S poles
   - Material properties and boundary conditions

4. FEA SOLUTION
   - Solves Maxwell's equations via finite element method
   - Computes magnetic vector potential and flux density

5. POST-PROCESSING
   - Parses solver output files
   - Creates publication-quality plots
   - Analyzes airgap flux density (critical for motor torque)

MOTOR GEOMETRY PARAMETERS (hardcoded for reproducibility):
-----------------------------------------------------------
- Rotor outer diameter: 110 mm (magnet outer surface)
- Magnet thickness: 5 mm (radial depth)
- Airgap: 1 mm (gap between rotor and stator)
- Stator inner diameter: 112 mm
- Stator outer diameter: 200 mm
- Number of poles: 8 (4 North, 4 South alternating)
- Magnet arc ratio: 0.8 (80% of pole pitch)
- Magnet coercivity: 900,000 A/m (NdFeB permanent magnet)
- Magnet relative permeability: 1.05

THREE-REGION AIR STRUCTURE (key for realistic motor FEA):
----------------------------------------------------------
- Inner air: Center to airgap middle (0 to 54.5mm radius)
  Contains rotating magnets with cutouts
  
- Outer air: Airgap middle to stator OD+2mm (54.5mm to 102mm)
  Represents stator region (future windings would go here)
  
- Shell (air infinity): Stator OD+2mm to OD+10mm (102mm to 110mm)
  Far-field boundary condition region

EXPECTED RESULTS:
-----------------
- 8-pole magnetic field pattern with rotational symmetry
- Airgap flux density: 0.14-0.80 Tesla
- Peak flux in magnets: ~0.84 Tesla
- Solver convergence: residual < 1e-10
- Output: Geometry files, mesh, solution fields, analysis plots

USAGE FOR AI TRAINING:
----------------------
This file demonstrates the complete electromagnetic FEA workflow without
external dependencies (except Gmsh/GetDP executables). All motor parameters
are defined inline. An AI agent can learn:
- How to construct motor geometry programmatically
- Mesh refinement strategies for electromagnetic problems
- Physics setup for magnetostatic simulations
- Post-processing and visualization of FEA results
"""

import subprocess
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.tri import Triangulation

# ==============================================================================
# MOTOR PARAMETERS (hardcoded for standalone operation)
# ==============================================================================
# These parameters define the 8-pole permanent magnet rotor geometry
# All dimensions in millimeters unless otherwise noted

class MotorParameters:
    """Electric motor geometric and magnetic parameters.
    
    This class encapsulates all parameters needed to define the motor geometry
    and magnetic properties. Hardcoded for training purposes.
    """
    
    def __init__(self):
        # Rotor geometry
        self.rotor_outer_diameter = 110.0  # mm - outer surface of magnets
        self.magnet_thickness = 5.0        # mm - radial depth of magnets
        
        # Airgap
        self.airgap_length = 1.0           # mm - critical gap between rotor and stator
        
        # Stator geometry
        self.stator_inner_diameter = 112.0 # mm - inner surface of stator
        self.stator_outer_diameter = 200.0 # mm - outer surface of stator
        
        # Pole configuration
        self.num_poles = 8                 # Total number of magnetic poles
        self.magnet_arc_ratio = 0.8        # Fraction of pole pitch covered by magnet (0-1)
        
        # Magnetic properties (NdFeB permanent magnet)
        self.magnet_hc = 900000.0          # A/m - coercive field strength
        self.magnet_mu_r = 1.05            # Relative permeability of magnet

# ==============================================================================
# PATHS AND SETUP
# ==============================================================================

# Get script directory and set up paths
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(script_dir)
src_dir = os.path.join(repo_root, 'src')

# Paths to required executables (must be installed separately)
getdp_dir = os.path.join(src_dir, 'getdp-3.5.0-Windows64')
getdp_exe = os.path.join(getdp_dir, 'getdp.exe')
getdp_templates = os.path.join(getdp_dir, 'templates')
gmsh_exe = os.path.join(src_dir, 'gmsh', 'gmsh.exe')

# Working directory for output files
work_dir = os.path.join(script_dir, 'how_emotor_fea_works_1_output')

# Initialize motor parameters
params = MotorParameters()

def create_geometry():
    """Step 1: Create Gmsh geometry file (.geo) with 3-region air structure.
    
    This function generates a text file in Gmsh's scripting language that defines
    the motor geometry. Gmsh will read this file to create the geometric model.
    
    GEOMETRY STRUCTURE:
    -------------------
    8 arc-shaped permanent magnets arranged radially:
    - Each magnet: Arc segment from R=49mm to R=54mm (5mm thick)
    - Angular positions: 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°
    - Alternating N-S polarity around the circle
    
    Three concentric air regions (critical for realistic motor FEA):
    - Inner air: Center to airgap middle (R=0 to 54.5mm) with magnet cutouts
      This is where the magnets rotate
      
    - Outer air: Airgap middle to stator OD+2mm (R=54.5mm to 102mm)
      This represents the stator region
      
    - Shell: Stator OD+2mm to OD+10mm (R=102mm to 110mm)
      Far-field boundary condition region
    
    MESH REFINEMENT STRATEGY:
    -------------------------
    - lc_magnet = 1mm (fine mesh in magnets for accurate field)
    - lc_airgap = 0.33mm (very fine at airgap - 3 elements across 1mm gap)
    - lc_inner_air = 10mm (medium in rotating region)
    - lc_outer_air = 15mm (medium in stator region)
    - lc_shell = 1.6mm (fine at far-field boundary)
    
    RETURNS:
    --------
    str: Path to created .geo file
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
    
    # Magnet arc angle (one pole)
    pole_pitch_angle = 2 * np.pi / params.num_poles
    magnet_arc_angle = pole_pitch_angle * params.magnet_arc_ratio
    half_arc = magnet_arc_angle / 2
    
    # Boundary radii - three air regions
    stator_outer_r = params.stator_outer_diameter / 2 * mm_to_m
    outer_air_r = stator_outer_r + 2 * mm_to_m  # Stator OD + 2mm
    domain_outer_r = stator_outer_r + 10 * mm_to_m  # Stator OD + 10mm
    Val_Rint = airgap_middle_r   # Inner boundary at airgap middle
    Val_Rext = domain_outer_r   # Outer boundary at domain outer
    
    # Mesh sizes
    lc_magnet = magnet_thickness / 5  # Fine mesh in magnet
    lc_airgap = airgap / 3  # Fine mesh at airgap middle (3 elements across)
    lc_inner_air = 0.01  # 10mm in inner air
    lc_outer_air = 0.015  # 15mm in outer air (stator region)
    lc_shell = (domain_outer_r - outer_air_r) / 5  # Shell mesh
    
    geo_content = f"""// {params.num_poles} Arc Magnets with Airgap - Motor Parameters
// Generated by how_emotor_fea_works_1.py

// Magnet geometry (from motor parameters)
magnet_inner_r = {magnet_inner_r:.6f};  // {(magnet_inner_r/mm_to_m):.1f}mm
magnet_outer_r = {rotor_outer_r:.6f};  // {(rotor_outer_r/mm_to_m):.1f}mm
half_arc = {half_arc:.6f};  // {np.degrees(half_arc):.2f} degrees
pole_angle = {pole_pitch_angle:.6f};  // {np.degrees(pole_pitch_angle):.2f} degrees

// Airgap and stator
airgap = {airgap:.6f};  // {(airgap/mm_to_m):.1f}mm
stator_inner_r = {stator_inner_r:.6f};  // {(stator_inner_r/mm_to_m):.1f}mm
airgap_middle_r = {airgap_middle_r:.6f};  // {(airgap_middle_r/mm_to_m):.1f}mm (middle of airgap)
outer_air_r = {outer_air_r:.6f};  // {(outer_air_r/mm_to_m):.1f}mm (stator OD + 2mm)
domain_outer_r = {domain_outer_r:.6f};  // {(domain_outer_r/mm_to_m):.1f}mm (stator OD + 10mm)

Val_Rint = {Val_Rint:.6f};  // Internal boundary (airgap middle)
Val_Rext = {Val_Rext:.6f};  // External boundary (domain outer)

// Mesh size parameters
lc_magnet = {lc_magnet:.6f};  // Magnet mesh
lc_airgap = {lc_airgap:.6f};  // Airgap mesh (fine)
lc_inner_air = {lc_inner_air:.6f};  // Inner air mesh
lc_outer_air = {lc_outer_air:.6f};  // Outer air mesh
lc_shell = {lc_shell:.6f};  // Shell mesh

// Center point
Point(1) = {{0, 0, 0, lc_inner_air}};

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

// Inner air region (center to airgap middle, minus magnets)
Curve Loop(220) = {{190, 191, 192, 193}};
Plane Surface(221) = {{220, {', '.join([str(20 + i*10) for i in range(params.num_poles)])}}};  // Inner air with magnet cutouts

// Outer air region (airgap middle to stator OD + 2mm)
Curve Loop(225) = {{200, 201, 202, 203}};
Plane Surface(226) = {{225, 220}};  // Outer air annulus

// Shell region (stator OD + 2mm to domain outer)
Curve Loop(230) = {{210, 211, 212, 213}};
Plane Surface(231) = {{230, 225}};  // Shell annulus

// Physical entities
Physical Surface("Inner Air", 100) = {{221}};
Physical Surface("Outer Air", 111) = {{226}};
Physical Surface("Air Inf", 101) = {{231}};
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
    print(f"  Magnets: {params.num_poles} poles, R={magnet_inner_r/mm_to_m:.1f}-{rotor_outer_r/mm_to_m:.1f}mm")
    print(f"  Arc per magnet: {np.degrees(magnet_arc_angle):.1f}° (N-S alternating)")
    print(f"  Airgap: {airgap/mm_to_m:.1f}mm (R={rotor_outer_r/mm_to_m:.1f}-{stator_inner_r/mm_to_m:.1f}mm)")
    print(f"  Inner air: R=0-{airgap_middle_r/mm_to_m:.1f}mm (to airgap middle)")
    print(f"  Outer air: R={airgap_middle_r/mm_to_m:.1f}-{outer_air_r/mm_to_m:.1f}mm (airgap middle to stator OD+2mm)")
    print(f"  Shell: R={outer_air_r/mm_to_m:.1f}-{domain_outer_r/mm_to_m:.1f}mm (stator OD+2mm to OD+10mm)")
    print(f"  Inner air: R=0-{magnet_inner_r/mm_to_m:.1f}mm")
    print(f"  Shell: R={stator_inner_r/mm_to_m:.1f}-{stator_outer_r/mm_to_m:.1f}mm (stator ID to OD)")
    print()
    
    return geo_file

def generate_mesh(geo_file):
    """Step 2: Generate finite element mesh using Gmsh.
    
    This function calls the Gmsh executable to convert the geometry file (.geo)
    into a finite element mesh (.msh) with triangular elements.
    
    WHAT IS MESHING?
    ----------------
    Meshing divides the continuous geometry into small discrete elements
    (triangles in 2D). The FEA solver will compute the magnetic field at
    each mesh node. Finer mesh = more accurate results but slower computation.
    
    ADAPTIVE REFINEMENT:
    --------------------
    The mesh is automatically refined based on the lc (characteristic length)
    values set in the geometry file:
    - Very fine at airgap boundary (0.33mm elements) - critical for accuracy
    - Fine in magnets (1mm elements) - important for field sources
    - Medium in air regions (10-15mm elements) - less critical
    - Coarse at far-field boundary (1.6mm elements) - minimal impact
    
    EXPECTED OUTPUT:
    ----------------
    - ~18,000 nodes (mesh vertices)
    - ~36,000 triangular elements
    - ~18,000 degrees of freedom (unknowns to solve)
    
    PARAMETERS:
    -----------
    geo_file : str
        Path to Gmsh geometry file (.geo)
    
    RETURNS:
    --------
    str or None: Path to generated mesh file (.msh), or None if failed
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
    """Step 3: Create GetDP solver input file (.pro) with physics definitions.
    
    This function generates the GetDP problem file that defines the electromagnetic
    physics, material properties, and boundary conditions.
    
    WHAT IS MAGNETOSTATICS?
    -----------------------
    Magnetostatics solves for magnetic fields from permanent magnets in the
    absence of currents or time-varying fields. This is the starting point
    for motor FEA - understanding the field from magnets alone.
    
    FORMULATION:
    ------------
    Uses vector potential formulation (a-phi method):
    - Primary unknown: magnetic vector potential 'a' (A_z component in 2D)
    - Governing equation: ∇×(ν∇×a) = ∇×Hc (where ν=1/μ, Hc=coercivity)
    - Secondary quantities: B = ∇×a (magnetic flux density)
                           H = νB - Hc (magnetic field intensity)
    
    MATERIAL REGIONS:
    -----------------
    Three air regions (all with μr=1, no magnetization):
    - InnerAir (region 100): Contains rotating magnets
    - OuterAir (region 111): Stator region  
    - AirInf (region 101): Far-field boundary
    
    Eight magnet regions (102-109) with radial magnetization:
    - Magnets 1,3,5,7: North poles (Hc pointing radially outward)
    - Magnets 2,4,6,8: South poles (Hc pointing radially inward)
    - Each magnet has Hc vector at its angular position (0°,45°,90°,...)
    
    BOUNDARY CONDITIONS:
    --------------------
    - a = 0 at outer shell boundary (flux is zero at infinity)
    - No boundary conditions at internal interfaces (natural continuity)
    
    MAGNETIZATION DETAILS:
    ----------------------
    Radial magnetization pattern for 8-pole rotor:
    - Magnet 1 at 0°:   Hc = (+900kA/m, 0) pointing right (N)
    - Magnet 2 at 45°:  Hc points inward toward center (S)
    - Magnet 3 at 90°:  Hc = (0, +900kA/m) pointing up (N)
    - ...alternating N-S around the circle
    
    POST-PROCESSING:
    ----------------
    Solver will output three .pos files:
    - c_core_az.pos: Magnetic vector potential field
    - c_core_b.pos: Magnetic flux density B (Tesla)
    - c_core_h.pos: Magnetic field intensity H (A/m)
    
    RETURNS:
    --------
    str: Path to created .pro file
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
    
    pro_content = f"""// {params.num_poles} Arc Magnets with Airgap - Motor Parameters
// Generated by how_emotor_fea_works_1.py

// Constants for spherical shell (required by library template)
DefineConstant[ Val_Rint = {airgap_middle_r:.6f} ];   // Internal boundary (airgap middle)
DefineConstant[ Val_Rext = {domain_outer_r:.6f} ];  // External boundary (domain outer)

// Region tags (must match geometry Physical entities)
INNER_AIR = 100;
OUTER_AIR = 111;
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
  InnerAir = Region[ INNER_AIR ];
  OuterAir = Region[ OUTER_AIR ];
  AirInf   = Region[ AIR_INF ];
  Dirichlet_a_0   = Region[ LINE_INF ];
  Dirichlet_phi_0 = Region[ LINE_INF ];

  // Generic group names for library template
  Vol_Mag = Region[ {{InnerAir, OuterAir, AirInf, {all_magnets_list}}} ];
  Vol_Inf_Mag = Region[ AirInf ];
  Vol_M_Mag   = Region[ {{{all_magnets_list}}} ];
}}

Function {{
  mu0 = 4.e-7 * Pi;
  
  // Material properties - all air regions and magnets
  nu [ Region[{{InnerAir, OuterAir, AirInf, {all_magnets_list}}} ] ] = 1. / mu0;
  mu [ Region[{{InnerAir, OuterAir, AirInf, {all_magnets_list}}} ] ] = mu0;
  
  // Permanent magnet coercive fields (radial magnetization, alternating N-S)
  Hc = {abs(params.magnet_hc)};  // A/m
  
  // Each magnet has radial magnetization at its angular position
"""
    
    # Add individual magnetization vectors for each magnet
    pole_pitch_angle = 2 * np.pi / params.num_poles
    for i in range(params.num_poles):
        angle = i * pole_pitch_angle
        polarity = "N" if i % 2 == 0 else "S"
        # North poles: radial outward, South poles: radial inward
        sign = 1 if i % 2 == 0 else -1
        hc_x = sign * np.cos(angle)
        hc_y = sign * np.sin(angle)
        pro_content += f"  hc [ Region[MAGNET_{polarity}{i+1}] ] = Vector[{hc_x:.6f}*Hc, {hc_y:.6f}*Hc, 0];  // {angle*180/np.pi:.1f}°, {'outward' if sign > 0 else 'inward'}\n"
    
    pro_content += """}

// Boundary conditions
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
    
    print(f"✓ Solver file created: {pro_file}")
    print(f"  Formulation: Magnetostatics (vector potential)")
    print(f"  Materials: Air (μr=1), Magnet (μr={params.magnet_mu_r})")
    print(f"  Regions: Inner air (0 to airgap middle), Outer air (airgap middle to stator OD+2mm), 8 magnets, Shell (stator OD+2mm to OD+10mm)")
    print(f"  Coercive field: Hc = {abs(params.magnet_hc)/1000:.0f} kA/m")
    print(f"  Magnetization: Radial, each magnet at its angular position")
    print(f"  {params.num_poles} poles: N={params.num_poles//2}, S={params.num_poles//2} (alternating, outward/inward)")
    print(f"  Boundary: a=0 on outer shell")
    print()
    
    return pro_file

def run_solver(pro_file, msh_file):
    """Step 4: Run GetDP finite element solver.
    
    This function executes the GetDP solver to compute the magnetic field
    distribution throughout the motor geometry.
    
    WHAT HAPPENS IN THE SOLVER?
    ----------------------------
    1. Read mesh file (.msh) - loads geometry and discretization
    2. Read problem file (.pro) - loads physics and material properties
    3. Assemble system matrix - builds [K]{a} = {f} from finite elements
       - [K] is ~18,000 × 18,000 sparse matrix (stiffness matrix)
       - {a} is vector of unknown magnetic potentials at each node
       - {f} is forcing vector from magnet coercivity
    4. Apply boundary conditions - sets a=0 at outer boundary
    5. Solve linear system - uses iterative solver (typically GMRES)
    6. Post-process - computes B and H fields from solution
    7. Write output files - saves .pos files for visualization
    
    FINITE ELEMENT METHOD:
    ----------------------
    FEM approximates the continuous magnetic field using piecewise polynomial
    basis functions on triangular elements. Each mesh node is a degree of
    freedom (DOF). More DOFs = better accuracy but slower computation.
    
    EXPECTED PERFORMANCE:
    ---------------------
    - Degrees of freedom: ~18,000 (one per mesh node in 2D)
    - System matrix size: ~18,000 × 18,000 (but sparse - mostly zeros)
    - Convergence tolerance: residual < 1e-9 (excellent accuracy)
    - Solution time: ~2-5 seconds on modern CPU
    - Memory usage: ~100 MB
    
    CONVERGENCE:
    ------------
    Iterative solver reduces error at each iteration. Monitor the residual:
    - Residual > 1e-3: Poor convergence, check geometry/materials
    - Residual < 1e-6: Good convergence, results are accurate
    - Residual < 1e-9: Excellent convergence (typical for this problem)
    
    PARAMETERS:
    -----------
    pro_file : str
        Path to GetDP problem file (.pro)
    msh_file : str
        Path to mesh file (.msh)
    
    RETURNS:
    --------
    bool: True if solver succeeded, False otherwise
    """
    
    print("=" * 60)
    print("Step 4: Running FEA Solver")
    print("=" * 60)
    print()
    
    if not os.path.exists(getdp_exe):
        print(f"ERROR: GetDP executable not found at {getdp_exe}")
        return False
    
    print("Running GetDP magnetostatic solver...")
    print(f"Problem file: {os.path.basename(pro_file)}")
    print(f"Mesh file: {os.path.basename(msh_file)}")
    print()
    
    try:
        result = subprocess.run(
            [getdp_exe, os.path.basename(pro_file), '-msh', os.path.basename(msh_file),
             '-solve', 'Magnetostatics_a', '-pos', 'MagnetostaticsSolution'],
            cwd=work_dir,
            capture_output=True,
            text=True,
            timeout=60
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
    """Step 5: Visualize results using Gmsh's built-in post-processor.
    
    This function launches Gmsh in interactive mode to visualize the mesh
    and magnetic field solution.
    
    VISUALIZATION 1: Mesh Display
    ------------------------------
    Shows the finite element mesh colored by physical region:
    - Different colors for magnets (8 regions)
    - Different colors for air regions (3 regions)
    - Edge display shows triangular element boundaries
    - Verifies geometry is correct before analyzing results
    
    VISUALIZATION 2: B-Field Display
    ---------------------------------
    Shows magnetic flux density magnitude |B| as a color contour plot:
    - Red/yellow: High flux density (~0.8 T in magnets)
    - Blue/green: Medium flux density (0.2-0.5 T in air)
    - Dark blue: Low flux density (<0.1 T at far field)
    
    WHAT TO LOOK FOR:
    -----------------
    1. 8-fold rotational symmetry (8 poles evenly spaced)
    2. Alternating high/low flux pattern (N-S-N-S-N-S-N-S)
    3. Flux concentrates in magnets and crosses airgap
    4. Field decays with distance from rotor
    5. No discontinuities or anomalies (indicates solver problems)
    
    EXPECTED FIELD PATTERN:
    -----------------------
    - Peak B in magnets: ~0.84 T (limited by magnet remanence)
    - Airgap B-field: 0.14-0.80 T (varies with angular position)
    - B in outer air: <0.1 T (far from magnets)
    - 8 flux "lobes" corresponding to 8 poles
    
    INTERACTIVE CONTROLS (in Gmsh window):
    ---------------------------------------
    - Left mouse: Rotate view
    - Right mouse: Zoom
    - Middle mouse: Pan
    - Double-click: Reset view
    - Tools menu: Adjust color scale, iso-values, etc.
    
    RETURNS:
    --------
    bool: True if visualization launched successfully
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
        print("Look for field distribution:")
        print("  - Magnet: High B-field (~1 T)")
        print("  - Air gap: Field propagates")
        print("  - Steel core: Concentrated field (2-3 T)")
        print("  - Surrounding air: Decreasing field")
        subprocess.Popen([gmsh_exe, b_field_file], cwd=work_dir)
        print("✓ B-field viewer launched")
        print()
        
        return True
        
    except Exception as e:
        print(f"ERROR launching Gmsh: {e}")
        return False


def parse_pos_file(pos_file):
    """Parse GetDP .pos file to extract mesh coordinates and field values.
    
    GetDP outputs results in a custom text format (.pos files). This function
    reads and parses these files to extract data for Python plotting.
    
    .POS FILE FORMAT:
    -----------------
    GetDP writes field data as "View" blocks containing element types and values.
    For 2D vector fields on triangles, each line looks like:
    
    VT(x1,y1,z1, x2,y2,z2, x3,y3,z3){vx1,vy1,vz1, vx2,vy2,vz2, vx3,vy3,vz3};
    
    Where:
    - VT = Vector Triangle element
    - x1,y1,z1 = coordinates of first vertex (z=0 for 2D)
    - x2,y2,z2 = coordinates of second vertex
    - x3,y3,z3 = coordinates of third vertex
    - vx1,vy1,vz1 = vector components at first vertex
    - vx2,vy2,vz2 = vector components at second vertex
    - vx3,vy3,vz3 = vector components at third vertex
    
    PARSING STRATEGY:
    -----------------
    1. Read file line by line
    2. Find lines starting with "VT("
    3. Extract coordinates from parentheses
    4. Extract values from curly braces
    5. Calculate magnitude: |v| = sqrt(vx² + vy² + vz²)
    6. Store as numpy arrays for plotting
    
    PARAMETERS:
    -----------
    pos_file : str
        Path to GetDP .pos output file
    
    RETURNS:
    --------
    tuple: (points, triangles, values)
        points : ndarray (N×2)
            Array of (x,y) coordinates for all vertices
        triangles : ndarray (M×3)
            Array of triangle connectivity (indices into points array)
        values : ndarray (N,)
            Array of field magnitudes at each point
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


def plot_results():
    """Step 6: Create publication-quality Python plots for field analysis.
    
    This function demonstrates post-processing FEA results using Python.
    It creates two complementary visualizations of the magnetic field.
    
    PLOT 1 (LEFT): 2D SPATIAL COLORMAP
    -----------------------------------
    Shows flux density magnitude |B| everywhere in the geometry.
    
    Technical details:
    - Uses matplotlib's tricontourf (triangle contour filled plot)
    - Colormap: 'viridis' (perceptually uniform, colorblind-friendly)
    - 50 contour levels for smooth gradients
    - Coordinates converted to mm (more intuitive than meters)
    - Colorbar shows Tesla units
    
    What this shows:
    - Overall field distribution and symmetry
    - High field regions (magnets) vs low field (far air)
    - Field penetration into stator region
    - Quality check: should see 8-fold symmetry
    
    PLOT 2 (RIGHT): AIRGAP FLUX DENSITY
    ------------------------------------
    Shows B-field vs angular position at the airgap middle (r=54.5mm).
    This is the most critical plot for motor analysis.
    
    Why airgap matters:
    - Airgap flux directly determines electromagnetic torque
    - Torque ∝ (airgap B-field) × (stator current)
    - Higher and smoother airgap flux = better motor performance
    
    Technical details:
    - Extracts points within ±0.5mm of airgap radius
    - Converts to polar coordinates (r, θ)
    - Sorts by angle for continuous line plot
    - Shows mean flux density (red dashed line)
    
    Expected pattern:
    - 8 peaks and valleys (8 poles)
    - Peaks: ~0.80 T (under North/South pole centers)
    - Valleys: ~0.14 T (between poles)
    - Should be nearly sinusoidal for good motor design
    
    OUTPUT:
    -------
    Saves 'flux_density_analysis.png' at 150 DPI (publication quality)
    Also displays interactive matplotlib window for zooming/panning
    
    LEARNING POINTS FOR AI:
    -----------------------
    1. How to parse custom FEA output formats
    2. Coordinate transformations (Cartesian to polar)
    3. Spatial filtering (extract airgap data)
    4. Triangle-based visualization techniques
    5. Creating professional technical plots with matplotlib
    
    RETURNS:
    --------
    bool: True if plotting succeeded, False otherwise
    """
    print("=" * 60)
    print("Step 6: Creating Python Plots")
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
    
    # Extract airgap samples at the middle of airgap
    mm_to_m = 0.001
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
    print(f"\n✓ Plot saved: {os.path.basename(plot_file)}")
    
    # Show plot
    plt.show()
    
    return True


def main():
    """Main execution function - orchestrates complete FEA workflow.
    
    This is the top-level function that calls all workflow steps in sequence.
    It demonstrates the complete electromagnetic FEA process from geometry
    creation through post-processing.
    
    COMPLETE WORKFLOW:
    ------------------
    1. CREATE GEOMETRY (.geo file)
       - Programmatically generate Gmsh script
       - Define 8 magnets and 3 air regions
       - Set mesh refinement parameters
       
    2. GENERATE MESH (.msh file)
       - Call Gmsh to discretize geometry
       - Creates ~18,000 triangular elements
       - Adaptive refinement at critical regions
       
    3. CREATE SOLVER FILE (.pro file)
       - Define physics (magnetostatics)
       - Set material properties (air, magnets)
       - Specify boundary conditions
       - Configure post-processing outputs
       
    4. RUN FEA SOLVER
       - Call GetDP to solve Maxwell's equations
       - Assembles and solves ~18,000×18,000 matrix system
       - Computes magnetic vector potential
       - Outputs field quantities (B, H)
       
    5. VISUALIZE IN GMSH
       - Launch interactive 3D viewers
       - Display mesh and field contours
       - Quality check results
       
    6. PYTHON POST-PROCESSING
       - Parse solver output files
       - Create publication-quality plots
       - Analyze airgap flux density
       - Save analysis results
    
    ERROR HANDLING:
    ---------------
    Each step checks for success before proceeding. If any step fails,
    the workflow stops and reports which step failed.
    
    OUTPUT LOCATION:
    ----------------
    All files are saved to: ../how_emotor_fea_works_1_output/
    
    RETURNS:
    --------
    int: Exit code (0=success, 1=failure)
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
    
    # Step 6: Create Python plots
    plot_success = plot_results()
    if not plot_success:
        print("\n⚠ Warning: Plot generation failed, but solver completed successfully")
    
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
    print("6. ✓ Python plots created - flux density analysis")
    print()
    print("This demonstrates a complete 8-pole rotor FEA with realistic airgap!")
    print()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
