"""
Magnetostatic FEA Solver
Solves 2D magnetostatic problem using GetDP
"""

import sys
import os
import numpy as np
from typing import Optional, Dict
import subprocess

# Paths to GetDP
GETDP_PATH = os.path.join(os.path.dirname(__file__), 'getdp-3.5.0-Windows64')
GETDP_EXE = os.path.join(GETDP_PATH, 'getdp.exe')
GETDP_TEMPLATES = os.path.join(GETDP_PATH, 'templates')
GMSH_EXE = os.path.join(os.path.dirname(__file__), 'gmsh', 'gmsh.exe')

from part_classes import Motor


class MagnetostaticSolver:
    """Solves 2D magnetostatic problem for motor cross-section"""
    
    def __init__(self, motor: Motor, mesh_file: str):
        """
        Initialize magnetostatic solver
        
        Args:
            motor: Motor object with material properties
            mesh_file: Path to .msh mesh file
        """
        self.motor = motor
        self.mesh_file = mesh_file
        self.pro_file = None
        self.res_file = None
        self.pos_file = None
        
    def solve(self, output_dir: Optional[str] = None) -> str:
        """
        Solve magnetostatic problem
        
        Args:
            output_dir: Directory for output files (default: data directory)
            
        Returns:
            Path to solution .pos file for visualization
        """
        if output_dir is None:
            output_dir = os.path.join(os.path.dirname(__file__), '..', 'data')
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate GetDP problem file
        self.pro_file = os.path.join(output_dir, 'motor_magnetostatic.pro')
        self._write_pro_file(self.pro_file)
        
        # Run GetDP solver
        self.res_file = os.path.join(output_dir, 'motor_magnetostatic.res')
        self.pos_file = os.path.join(output_dir, 'motor_magnetostatic.pos')
        self._run_getdp()
        
        return self.pos_file
    
    def _write_pro_file(self, filepath: str):
        """Write GetDP problem definition file using library template"""
        
        # Get material properties
        stator = self.motor.stator
        rotor = self.motor.rotor
        
        # Permeability values (relative)
        mu_steel = 200.0  # Typical for electrical steel (matching C-core example)
        mu_magnet = 1.05   # Typical for NdFeB
        
        # Magnet coercivity (A/m)
        hc_magnet = 920000  # Typical for NdFeB N42 (Br ~1.2T)
        
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write("/* 2D Magnetostatic problem for motor cross-section */\n")
            f.write("/* Uses GetDP library template for magnetostatics */\n\n")
            
            # Physical region tags from mesh
            f.write("// Region tags from mesh\n")
            f.write("OUTER_AIR = 1;\n")
            f.write("STATOR_YOKE = 2;\n")
            f.write("STATOR_TEETH = 3;\n")
            f.write("SLOTS = 4;\n")
            f.write("AIRGAP = 5;\n")
            f.write("MAGNETS = 6;\n")
            f.write("AIR_BETWEEN_MAGNETS = 7;\n")
            f.write("ROTOR = 8;\n")
            f.write("SHAFT = 9;\n")
            f.write("OUTER_BOUNDARY = 1001;\n\n")
            
            # Groups for library template
            f.write("Group {\n")
            f.write("  // Individual regions from mesh\n")
            f.write("  OuterAir = Region[OUTER_AIR];\n")
            f.write("  StatorYoke = Region[STATOR_YOKE];\n")
            f.write("  StatorTeeth = Region[STATOR_TEETH];\n")
            f.write("  Slots = Region[SLOTS];\n")
            f.write("  Airgap = Region[AIRGAP];\n")
            f.write("  Magnets = Region[MAGNETS];\n")
            f.write("  AirBetweenMagnets = Region[AIR_BETWEEN_MAGNETS];\n")
            f.write("  Rotor = Region[ROTOR];\n")
            f.write("  Shaft = Region[SHAFT];\n")
            f.write("  OuterBoundary = Region[OUTER_BOUNDARY];\n\n")
            
            f.write("  // Material groupings\n")
            f.write("  Air = Region[{OuterAir, Slots, Airgap, AirBetweenMagnets}];\n")
            f.write("  Steel = Region[{StatorYoke, StatorTeeth, Rotor, Shaft}];\n")
            f.write("  MagnetRegion = Region[{Magnets}];\n\n")
            
            f.write("  // Boundary condition\n")
            f.write("  Dirichlet_a_0 = Region[OuterBoundary];\n")
            f.write("  Dirichlet_phi_0 = Region[OuterBoundary];\n\n")
            
            f.write("  // Generic group names required by library template\n")
            f.write("  Vol_Mag = Region[{Air, Steel, MagnetRegion}];\n")
            f.write("  Vol_M_Mag = Region[MagnetRegion];\n")
            f.write("}\n\n")
            
            # Functions
            f.write("Function {\n")
            f.write("  mu0 = 4.e-7 * Pi;\n\n")
            
            f.write("  // Material properties\n")
            f.write("  nu[Air] = 1. / mu0;\n")
            f.write("  mu[Air] = mu0;\n\n")
            
            f.write(f"  // Steel: μr = {mu_steel}\n")
            f.write(f"  nu[Steel] = 1. / ({mu_steel} * mu0);\n")
            f.write(f"  mu[Steel] = {mu_steel} * mu0;\n\n")
            
            f.write(f"  // Magnets: μr = {mu_magnet}\n")
            f.write(f"  nu[MagnetRegion] = 1. / ({mu_magnet} * mu0);\n")
            f.write(f"  mu[MagnetRegion] = {mu_magnet} * mu0;\n\n")
            
            f.write("  // Permanent magnet coercive field (radial magnetization)\n")
            f.write(f"  Hc = {hc_magnet};  // A/m\n")
            f.write("  r[] = Sqrt[X[]*X[] + Y[]*Y[]];\n")
            f.write("  hc[MagnetRegion] = Hc / r[] * Vector[X[], Y[], 0.];\n")
            f.write("}\n\n")
            
            # Boundary conditions
            f.write("Constraint {\n")
            f.write("  { Name a;\n")
            f.write("    Case {\n")
            f.write("      { Region Dirichlet_a_0; Value 0.; }\n")
            f.write("    }\n")
            f.write("  }\n")
            f.write("  { Name phi;\n")
            f.write("    Case {\n")
            f.write("      { Region Dirichlet_phi_0; Value 0.; }\n")
            f.write("    }\n")
            f.write("  }\n")
            f.write("}\n\n")
            
            # Include library template - THIS IS THE KEY!
            f.write("// Include GetDP magnetostatic formulation library\n")
            f.write(f'Include "{GETDP_TEMPLATES}/Lib_Magnetostatics_a_phi.pro"\n\n')
            
            # Post-processing
            f.write("// Post-processing operations\n")
            f.write("PostOperation {\n")
            f.write("  { Name MagnetostaticsSolution; NameOfPostProcessing Magnetostatics_a;\n")
            f.write("    Operation {\n")
            f.write("      Print[ az, OnElementsOf Vol_Mag, File \"motor_magnetostatic_a.pos\" ];\n")
            f.write("      Print[ b, OnElementsOf Vol_Mag, File \"motor_magnetostatic_b.pos\" ];\n")
            f.write("      Print[ h, OnElementsOf Vol_Mag, File \"motor_magnetostatic_h.pos\" ];\n")
            f.write("    }\n")
            f.write("  }\n")
            f.write("}\n")
    
    def _run_getdp(self):
        """Run GetDP solver"""
        if not os.path.exists(GETDP_EXE):
            raise FileNotFoundError(f"GetDP executable not found at {GETDP_EXE}")
        
        if self.pro_file is None:
            raise RuntimeError("Problem file not generated yet")
        
        # Change to output directory for GetDP execution
        output_dir = os.path.dirname(self.pro_file)
        original_dir = os.getcwd()
        
        try:
            os.chdir(output_dir)
            
            # Run GetDP: getdp problem.pro -solve Resolution -pos PostOperation
            cmd = [GETDP_EXE, os.path.basename(self.pro_file), 
                   "-msh", os.path.abspath(self.mesh_file),
                   "-solve", "Magnetostatics_a",  # Use library template resolution name
                   "-pos", "MagnetostaticsSolution"]  # Use our post-operation name
            
            print(f"\nRunning GetDP solver...")
            print(f"Command: {' '.join(cmd)}")
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode != 0:
                print("GetDP stdout:", result.stdout)
                print("GetDP stderr:", result.stderr)
                raise RuntimeError(f"GetDP solver failed with return code {result.returncode}")
            
            print("GetDP solver completed successfully")
            
        finally:
            os.chdir(original_dir)
    
    def view_results(self, field: str = "b"):
        """
        Open results in gmsh viewer
        
        Args:
            field: Field to view ('a', 'b', 'h', or 'norm_b')
        """
        if not os.path.exists(GMSH_EXE):
            raise FileNotFoundError(f"Gmsh executable not found at {GMSH_EXE}")
        
        if self.pro_file is None:
            raise RuntimeError("Problem file not generated yet")
        
        # Determine which .pos file to open
        output_dir = os.path.dirname(self.pro_file)
        
        pos_files = []
        if field == "a":
            pos_files.append(os.path.join(output_dir, "motor_magnetostatic_a.pos"))
        elif field == "b":
            pos_files.append(os.path.join(output_dir, "motor_magnetostatic_b.pos"))
        elif field == "h":
            pos_files.append(os.path.join(output_dir, "motor_magnetostatic_h.pos"))
        elif field == "norm_b":
            # For norm_b, just use the b-field and let Gmsh calculate magnitude
            pos_files.append(os.path.join(output_dir, "motor_magnetostatic_b.pos"))
            # Also load A-field for flux line visualization
            a_file = os.path.join(output_dir, "motor_magnetostatic_a.pos")
            if os.path.exists(a_file):
                pos_files.append(a_file)
        else:
            raise ValueError(f"Unknown field: {field}")
        
        for pos_file in pos_files:
            if not os.path.exists(pos_file):
                print(f"Warning: Solution file not found: {pos_file}")
        
        # Open in gmsh with mesh and all post files
        print(f"\nOpening {field} field in gmsh viewer...")
        if len(pos_files) > 1:
            print(f"  - Including A-field (az) for flux line visualization")
            print(f"  - In Gmsh: Tools > Plugins > Isosurface to create A-field contours")
        subprocess.Popen([GMSH_EXE, self.mesh_file] + pos_files)


def solve_magnetostatic(motor: Motor, mesh_file: str, view_field: str = "norm_b") -> str:
    """
    Convenience function to solve magnetostatic problem and view results
    
    Args:
        motor: Motor object
        mesh_file: Path to mesh file
        view_field: Field to view in gmsh ('a', 'b', 'h', 'norm_b')
        
    Returns:
        Path to solution .pos file
    """
    solver = MagnetostaticSolver(motor, mesh_file)
    pos_file = solver.solve()
    solver.view_results(view_field)
    
    return pos_file
