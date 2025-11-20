"""
Finite Element Analysis (FEA) solver for electromagnetic analysis
Uses gmsh for meshing and GetDP for magnetostatic field solution
"""
import numpy as np
import gmsh
import os
from typing import Dict, Tuple
from motor_parameters_v0_1 import MotorParameters
from geometry_v0_1 import MotorGeometry


class FEASolver:
    """
    FEA-based electromagnetic solver using gmsh for mesh generation
    and GetDP for magnetostatic field solution
    """
    
    def __init__(self, params: MotorParameters, geometry: MotorGeometry):
        self.params = params
        self.geometry = geometry
        self.mesh_size = 2.0  # Default mesh size in mm
        self._last_mesh_file = None  # Store path to last generated mesh file
        self._last_temp_dir = None  # Store temp directory for cleanup
        
    def create_mesh(self, mesh_size: float = None) -> Dict:
        """
        Create 2D mesh using gmsh for the motor cross-section with actual geometry
        
        Args:
            mesh_size: Characteristic mesh size in mm (None = use default)
            
        Returns:
            Dictionary with mesh data (nodes, elements, regions)
        """
        if mesh_size is not None:
            self.mesh_size = mesh_size
            
        # Get actual geometry from MotorGeometry
        stator_geo = self.geometry.generate_stator_geometry()
        rotor_geo = self.geometry.generate_rotor_geometry()
            
        # Initialize gmsh
        gmsh.initialize()
        gmsh.model.add("motor_2d")
        
        # CONVERT FROM MM TO METERS for FEA (SI units)
        # Motor parameters are in mm, GetDP expects meters
        mm_to_m = 0.001
        
        # Set mesh size (also in meters)
        gmsh.option.setNumber("Mesh.CharacteristicLengthMin", self.mesh_size * mm_to_m / 2)
        gmsh.option.setNumber("Mesh.CharacteristicLengthMax", self.mesh_size * mm_to_m * 2)
        
        # Radii (convert mm to m)
        r_shaft = self.params.rotor_inner_diameter / 2 * mm_to_m
        r_rotor = self.params.rotor_outer_diameter / 2 * mm_to_m
        r_stator_bore = self.params.stator_inner_diameter / 2 * mm_to_m
        r_stator_outer = self.params.stator_outer_diameter / 2 * mm_to_m
        
        # Create geometry regions
        # 1. Shaft (center)
        shaft_circle = gmsh.model.occ.addCircle(0, 0, 0, r_shaft)
        shaft_loop = gmsh.model.occ.addCurveLoop([shaft_circle])
        shaft_surface = gmsh.model.occ.addPlaneSurface([shaft_loop])
        
        # 2. Rotor core (will subtract magnets)
        rotor_circle = gmsh.model.occ.addCircle(0, 0, 0, r_rotor)
        rotor_loop = gmsh.model.occ.addCurveLoop([rotor_circle])
        rotor_surface = gmsh.model.occ.addPlaneSurface([rotor_loop, shaft_loop])
        
        # 3. Create individual magnet surfaces from actual geometry
        magnet_surfaces = []
        for magnet in rotor_geo['magnets']:
            vertices = magnet['vertices']
            # Create magnet polygon (convert vertices to meters)
            points = []
            for x, y in vertices:
                points.append(gmsh.model.occ.addPoint(x * mm_to_m, y * mm_to_m, 0))
            
            # Create lines connecting points
            lines = []
            for i in range(len(points)):
                lines.append(gmsh.model.occ.addLine(points[i], points[(i+1) % len(points)]))
            
            # Create curve loop and surface
            magnet_loop = gmsh.model.occ.addCurveLoop(lines)
            magnet_surf = gmsh.model.occ.addPlaneSurface([magnet_loop])
            magnet_surfaces.append(magnet_surf)
        
        # 4. Stator with slots properly subtracted to form teeth
        # First create individual slot surfaces from actual geometry
        slot_surfaces = []
        for slot in stator_geo['slots']:
            vertices = slot['vertices']
            # Create slot polygon (convert vertices to meters)
            points = []
            for x, y in vertices:
                points.append(gmsh.model.occ.addPoint(x * mm_to_m, y * mm_to_m, 0))
            
            # Create lines connecting points
            lines = []
            for i in range(len(points)):
                lines.append(gmsh.model.occ.addLine(points[i], points[(i+1) % len(points)]))
            
            # Create curve loop and surface
            slot_loop = gmsh.model.occ.addCurveLoop(lines)
            slot_surf = gmsh.model.occ.addPlaneSurface([slot_loop])
            slot_surfaces.append(slot_surf)
        
        # Now create the full stator annulus
        stator_circle = gmsh.model.occ.addCircle(0, 0, 0, r_stator_outer)
        stator_outer_loop = gmsh.model.occ.addCurveLoop([stator_circle])
        stator_bore_circle = gmsh.model.occ.addCircle(0, 0, 0, r_stator_bore)
        stator_bore_loop = gmsh.model.occ.addCurveLoop([stator_bore_circle])
        
        # Create stator annulus (before subtracting slots)
        stator_base = gmsh.model.occ.addPlaneSurface([stator_outer_loop, stator_bore_loop])
        
        # Boolean subtract all slots from stator to create teeth
        teeth_surfaces = []
        if slot_surfaces:
            # Use cut operation to subtract slots from stator, creating teeth
            # Cut returns: (outDimTags, outDimTagsMap)
            result_teeth, result_map = gmsh.model.occ.cut(
                [(2, stator_base)],
                [(2, s) for s in slot_surfaces],
                removeObject=True,
                removeTool=False
            )
            gmsh.model.occ.synchronize()
            
            # The cut operation returns the stator with slots removed (teeth)
            # Extract just the surface tags
            teeth_surfaces = [tag for dim, tag in result_teeth if dim == 2]
            
            # The slot_surfaces are preserved as the tool objects (windings)
        else:
            teeth_surfaces = [stator_base]
        
        # 5. Create airgap as the region between magnets and stator bore
        # This is the annulus not occupied by magnets or slots  
        # Add a middle circle for sampling to ensure we have nodes at the correct radius
        magnet_outer_r = r_rotor + self.params.magnet_thickness * mm_to_m
        r_airgap_sample = (magnet_outer_r + r_stator_bore) / 2  # Midpoint for sampling
        
        airgap_outer_circle = gmsh.model.occ.addCircle(0, 0, 0, r_stator_bore)
        airgap_inner_circle = gmsh.model.occ.addCircle(0, 0, 0, magnet_outer_r)
        
        # Add intermediate circles in the airgap for structured radial meshing
        # Create 3 radial layers by adding 2 intermediate circles
        airgap_thickness = r_stator_bore - magnet_outer_r
        middle_circle_1 = gmsh.model.occ.addCircle(0, 0, 0, magnet_outer_r + airgap_thickness/3)
        middle_circle_2 = gmsh.model.occ.addCircle(0, 0, 0, magnet_outer_r + 2*airgap_thickness/3)
        
        # Create curve loops
        airgap_outer_loop = gmsh.model.occ.addCurveLoop([airgap_outer_circle])
        middle_loop_2 = gmsh.model.occ.addCurveLoop([middle_circle_2])
        middle_loop_1 = gmsh.model.occ.addCurveLoop([middle_circle_1])
        airgap_inner_loop = gmsh.model.occ.addCurveLoop([airgap_inner_circle])
        
        # Create 3 annular surfaces (3 radial layers in the airgap)
        airgap_layer_1 = gmsh.model.occ.addPlaneSurface([middle_loop_1, airgap_inner_loop])
        airgap_layer_2 = gmsh.model.occ.addPlaneSurface([middle_loop_2, middle_loop_1])
        airgap_layer_3 = gmsh.model.occ.addPlaneSurface([airgap_outer_loop, middle_loop_2])
        
        # Synchronize CAD
        gmsh.model.occ.synchronize()
        
        # Apply transfinite meshing to airgap circles for consistent angular discretization
        gmsh.model.mesh.setTransfiniteCurve(airgap_outer_circle, 360)
        gmsh.model.mesh.setTransfiniteCurve(middle_circle_2, 360)
        gmsh.model.mesh.setTransfiniteCurve(middle_circle_1, 360)
        gmsh.model.mesh.setTransfiniteCurve(airgap_inner_circle, 360)
        
        # Create physical groups for material assignment
        gmsh.model.addPhysicalGroup(2, [shaft_surface], 1, "Shaft")
        gmsh.model.addPhysicalGroup(2, [rotor_surface], 2, "Rotor")
        
        # Create SEPARATE physical groups for each magnet with polarity info
        # Region 100-199 reserved for magnets (100 + pole_index)
        magnet_region_info = []
        if magnet_surfaces:
            for i, (magnet_surf, magnet_geo) in enumerate(zip(magnet_surfaces, rotor_geo['magnets'])):
                region_tag = 100 + i
                polarity = magnet_geo['polarity']
                angle = magnet_geo['angle']
                gmsh.model.addPhysicalGroup(2, [magnet_surf], region_tag, f"Magnet_{i}")
                magnet_region_info.append({'tag': region_tag, 'polarity': polarity, 'angle': angle})
        
        # Group all 3 airgap layers into single physical region for GetDP
        gmsh.model.addPhysicalGroup(2, [airgap_layer_1, airgap_layer_2, airgap_layer_3], 4, "Airgap")
        gmsh.model.addPhysicalGroup(2, teeth_surfaces, 5, "Stator")  # Teeth (iron)
        if slot_surfaces:
            gmsh.model.addPhysicalGroup(2, slot_surfaces, 6, "Windings")  # Slots (copper)
        
        # Generate 2D mesh (transfinite curves already set above)
        gmsh.model.mesh.generate(2)
        
        # Extract mesh data
        mesh_data = self._extract_mesh_data()
        
        # Store magnet region info for GetDP
        mesh_data['magnet_regions'] = magnet_region_info
        
        # Finalize gmsh
        gmsh.finalize()
        
        return mesh_data
    
    def _extract_mesh_data(self) -> Dict:
        """Extract mesh data from gmsh"""
        # Get nodes
        node_tags, node_coords, _ = gmsh.model.mesh.getNodes()
        nodes = np.array(node_coords).reshape(-1, 3)[:, :2]  # Only x, y for 2D
        
        # Get elements (triangles for 2D)
        element_types, element_tags, element_node_tags = gmsh.model.mesh.getElements(2)
        
        elements = []
        element_regions = []
        
        for elem_type, elem_tag_list, elem_nodes in zip(element_types, element_tags, element_node_tags):
            if elem_type == 2:  # Triangle
                nodes_per_elem = 3
                num_elems = len(elem_tag_list)
                elem_array = np.array(elem_nodes).reshape(num_elems, nodes_per_elem) - 1  # 0-indexed
                elements.append(elem_array)
                
                # Get region for each element
                for elem_tag in elem_tag_list:
                    entities = gmsh.model.mesh.getElement(int(elem_tag))
                    element_regions.append(entities[3])
        
        if elements:
            elements = np.vstack(elements)
        else:
            elements = np.array([])
            
        # Get physical group info
        physical_groups = {}
        for dim in [2]:  # 2D surfaces
            tags = gmsh.model.getPhysicalGroups(dim)
            for tag in tags:
                name = gmsh.model.getPhysicalName(dim, tag[1])
                entities = gmsh.model.getEntitiesForPhysicalGroup(dim, tag[1])
                physical_groups[name] = {'tag': tag[1], 'entities': entities}
        
        return {
            'nodes': nodes,
            'elements': elements,
            'element_regions': np.array(element_regions),
            'physical_groups': physical_groups,
            'num_nodes': len(nodes),
            'num_elements': len(elements)
        }
    
    def solve_magnetostatic(self, mesh_data: Dict, rotor_angle: float = 0.0) -> Dict:
        """
        Solve 2D magnetostatic problem using GetDP
        GetDP is gmsh's companion solver for electromagnetic field problems
        
        Args:
            mesh_data: Mesh data from create_mesh()
            rotor_angle: Rotor angle in degrees
            
        Returns:
            Solution dictionary with magnetic vector potential and flux densities
        """
        import os
        import tempfile
        import subprocess
        
        # Cleanup previous temp directory if exists
        if self._last_temp_dir and os.path.exists(self._last_temp_dir):
            import shutil
            shutil.rmtree(self._last_temp_dir, ignore_errors=True)
        
        # Create temporary directory for GetDP files
        temp_dir = tempfile.mkdtemp()
        self._last_temp_dir = temp_dir  # Store for cleanup on next solve
        mesh_file = os.path.join(temp_dir, "magnetostatic.msh")
        pro_file = os.path.join(temp_dir, "magnetostatic.pro")
        res_dir = os.path.join(temp_dir, "res")
        os.makedirs(res_dir, exist_ok=True)
        
        # Generate mesh with GetDP
        mesh_data_with_magnets = self._create_getdp_mesh(mesh_file, rotor_angle)
        
        # Create GetDP problem definition with magnet polarities
        self._create_getdp_problem(pro_file, rotor_angle, mesh_data_with_magnets.get('magnet_regions', []))
        
        # Run GetDP solver
        getdp_path = os.path.join(os.path.dirname(__file__), "getdp-3.5.0-Windows64", "getdp.exe")
        if not os.path.exists(getdp_path):
            getdp_path = "getdp"  # Try system PATH
        
        try:
            result = subprocess.run(
                [getdp_path, pro_file, "-solve", "MagSta", "-pos", "MagSta_a", "-v", "3"],
                cwd=temp_dir,
                capture_output=True,
                text=True,
                timeout=60
            )
            
            # DEBUG: Print GetDP output
            if result.stdout:
                print("GetDP stdout:")
                print(result.stdout[-500:])  # Last 500 chars
            if result.stderr:
                print("GetDP stderr:")
                print(result.stderr[-500:])
            
            if result.returncode != 0:
                print("GetDP solver failed:")
                print(result.stderr)
                raise RuntimeError(f"GetDP failed with return code {result.returncode}")
                
        except FileNotFoundError:
            print(f"GetDP not found at: {getdp_path}")
            print("Please install GetDP from https://getdp.info/")
            raise
        
        # Parse results
        solution = self._parse_getdp_results(temp_dir, mesh_data)
        
        # Store mesh file path before cleanup (mesh file is already set in _create_getdp_mesh)
        # Don't cleanup temp_dir yet - we need the mesh file for Gmsh viewer
        # The temp directory will be cleaned up on next solve or program exit
        
        return solution
    
    def _create_getdp_mesh(self, mesh_file: str, rotor_angle: float):
        """Generate gmsh mesh file with proper physical groups for GetDP"""
        # Regenerate mesh with GetDP-compatible format
        gmsh.initialize()
        gmsh.model.add("motor_getdp")
        
        # CONVERT FROM MM TO METERS for FEA (SI units)
        # Motor parameters are in mm, GetDP expects meters
        mm_to_m = 0.001
        
        # Set mesh size (also in meters)
        gmsh.option.setNumber("Mesh.CharacteristicLengthMin", self.mesh_size * mm_to_m / 2)
        gmsh.option.setNumber("Mesh.CharacteristicLengthMax", self.mesh_size * mm_to_m * 2)
        
        # Get geometry
        stator_geo = self.geometry.generate_stator_geometry()
        rotor_geo = self.geometry.generate_rotor_geometry()
        
        # Radii (convert mm to m)
        r_shaft = self.params.rotor_inner_diameter / 2 * mm_to_m
        r_rotor = self.params.rotor_outer_diameter / 2 * mm_to_m
        r_stator_bore = self.params.stator_inner_diameter / 2 * mm_to_m
        r_stator_outer = self.params.stator_outer_diameter / 2 * mm_to_m
        
        # Create all regions (same as create_mesh)
        shaft_circle = gmsh.model.occ.addCircle(0, 0, 0, r_shaft)
        shaft_loop = gmsh.model.occ.addCurveLoop([shaft_circle])
        shaft_surface = gmsh.model.occ.addPlaneSurface([shaft_loop])
        
        rotor_circle = gmsh.model.occ.addCircle(0, 0, 0, r_rotor)
        rotor_loop = gmsh.model.occ.addCurveLoop([rotor_circle])
        rotor_surface = gmsh.model.occ.addPlaneSurface([rotor_loop, shaft_loop])
        
        # Magnets (convert vertices to meters)
        magnet_surfaces = []
        for magnet in rotor_geo['magnets']:
            vertices = magnet['vertices']
            points = [gmsh.model.occ.addPoint(x * mm_to_m, y * mm_to_m, 0) for x, y in vertices]
            lines = [gmsh.model.occ.addLine(points[i], points[(i+1) % len(points)]) for i in range(len(points))]
            magnet_loop = gmsh.model.occ.addCurveLoop(lines)
            magnet_surf = gmsh.model.occ.addPlaneSurface([magnet_loop])
            magnet_surfaces.append(magnet_surf)
        
        # Slots (convert vertices to meters)
        slot_surfaces = []
        for slot in stator_geo['slots']:
            vertices = slot['vertices']
            points = [gmsh.model.occ.addPoint(x * mm_to_m, y * mm_to_m, 0) for x, y in vertices]
            lines = [gmsh.model.occ.addLine(points[i], points[(i+1) % len(points)]) for i in range(len(points))]
            slot_loop = gmsh.model.occ.addCurveLoop(lines)
            slot_surf = gmsh.model.occ.addPlaneSurface([slot_loop])
            slot_surfaces.append(slot_surf)
        
        # Stator
        stator_circle = gmsh.model.occ.addCircle(0, 0, 0, r_stator_outer)
        stator_outer_loop = gmsh.model.occ.addCurveLoop([stator_circle])
        stator_bore_circle = gmsh.model.occ.addCircle(0, 0, 0, r_stator_bore)
        stator_bore_loop = gmsh.model.occ.addCurveLoop([stator_bore_circle])
        stator_base = gmsh.model.occ.addPlaneSurface([stator_outer_loop, stator_bore_loop])
        
        teeth_surfaces = []
        if slot_surfaces:
            result_teeth, _ = gmsh.model.occ.cut([(2, stator_base)], [(2, s) for s in slot_surfaces],
                                                   removeObject=True, removeTool=False)
            gmsh.model.occ.synchronize()
            teeth_surfaces = [tag for dim, tag in result_teeth if dim == 2]
        else:
            teeth_surfaces = [stator_base]
        
        # Airgap - create 3 radial layers with structured mesh
        magnet_outer_r = r_rotor + self.params.magnet_thickness * mm_to_m
        r_airgap_sample = (magnet_outer_r + r_stator_bore) / 2
        
        airgap_outer_circle = gmsh.model.occ.addCircle(0, 0, 0, r_stator_bore)
        airgap_inner_circle = gmsh.model.occ.addCircle(0, 0, 0, magnet_outer_r)
        
        # Add intermediate circles for radial layers
        airgap_thickness = r_stator_bore - magnet_outer_r
        middle_circle_1 = gmsh.model.occ.addCircle(0, 0, 0, magnet_outer_r + airgap_thickness/3)
        middle_circle_2 = gmsh.model.occ.addCircle(0, 0, 0, magnet_outer_r + 2*airgap_thickness/3)
        
        # Create curve loops
        airgap_outer_loop = gmsh.model.occ.addCurveLoop([airgap_outer_circle])
        middle_loop_2 = gmsh.model.occ.addCurveLoop([middle_circle_2])
        middle_loop_1 = gmsh.model.occ.addCurveLoop([middle_circle_1])
        airgap_inner_loop = gmsh.model.occ.addCurveLoop([airgap_inner_circle])
        
        # Create 3 annular surfaces
        airgap_layer_1 = gmsh.model.occ.addPlaneSurface([middle_loop_1, airgap_inner_loop])
        airgap_layer_2 = gmsh.model.occ.addPlaneSurface([middle_loop_2, middle_loop_1])
        airgap_layer_3 = gmsh.model.occ.addPlaneSurface([airgap_outer_loop, middle_loop_2])
        
        gmsh.model.occ.synchronize()
        
        # Apply transfinite meshing to airgap circles for consistent angular discretization
        gmsh.model.mesh.setTransfiniteCurve(airgap_outer_circle, 360)
        gmsh.model.mesh.setTransfiniteCurve(middle_circle_2, 360)
        gmsh.model.mesh.setTransfiniteCurve(middle_circle_1, 360)
        gmsh.model.mesh.setTransfiniteCurve(airgap_inner_circle, 360)
        
        # Physical groups - create SEPARATE regions for each magnet
        gmsh.model.addPhysicalGroup(2, [shaft_surface], 1, "Shaft")
        gmsh.model.addPhysicalGroup(2, [rotor_surface], 2, "Rotor")
        
        # Create individual physical groups for each magnet with polarity info
        magnet_region_info = []
        if magnet_surfaces:
            for i, (magnet_surf, magnet_geo) in enumerate(zip(magnet_surfaces, rotor_geo['magnets'])):
                region_tag = 100 + i
                polarity = magnet_geo['polarity']
                angle = magnet_geo['angle']
                gmsh.model.addPhysicalGroup(2, [magnet_surf], region_tag, f"Magnet_{i}")
                magnet_region_info.append({'tag': region_tag, 'polarity': polarity, 'angle': angle})
        
        # Group all 3 airgap layers into single physical region for GetDP
        gmsh.model.addPhysicalGroup(2, [airgap_layer_1, airgap_layer_2, airgap_layer_3], 4, "Airgap")
        gmsh.model.addPhysicalGroup(2, teeth_surfaces, 5, "Stator")
        if slot_surfaces:
            gmsh.model.addPhysicalGroup(2, slot_surfaces, 6, "Windings")
        
        # Add boundary physical group (outer stator surface)
        # Find the outer boundary curve
        outer_curves = []
        for curve in gmsh.model.getEntities(1):
            bounds = gmsh.model.getBoundingBox(curve[0], curve[1])
            x_center = (bounds[0] + bounds[3]) / 2
            y_center = (bounds[1] + bounds[4]) / 2
            r = np.sqrt(x_center**2 + y_center**2)
            if abs(r - r_stator_outer) < 0.001:  # 1 mm tolerance in meters
                outer_curves.append(curve[1])
        
        if outer_curves:
            gmsh.model.addPhysicalGroup(1, outer_curves, 200, "OuterBoundary")
        
        # Generate mesh
        gmsh.model.mesh.generate(2)
        
        # Save mesh
        gmsh.write(mesh_file)
        
        # Store mesh file path for viewing in Gmsh
        self._last_mesh_file = mesh_file
        
        gmsh.finalize()
        
        # Return magnet region info for GetDP problem generation
        return {'magnet_regions': magnet_region_info}
    
    def _create_getdp_problem(self, pro_file: str, rotor_angle: float, magnet_regions: list):
        """Create GetDP problem definition (.pro file) for magnetostatics with individual magnet polarities"""
        
        mu_0 = 4 * np.pi * 1e-7
        mu_r_steel = self.params.steel_mu_r
        mu_r_magnet = self.params.magnet_mu_r
        magnet_br = self.params.magnet_br
        
        # Build magnet regions string and magnetization functions
        magnet_region_list = []
        magnet_nu_assignments = []
        magnet_hc_assignments = []
        
        for mag_info in magnet_regions:
            tag = mag_info['tag']
            polarity = mag_info['polarity']
            angle = mag_info['angle']
            magnet_region_list.append(str(tag))
            
            # Reluctivity for this magnet
            magnet_nu_assignments.append(f"  nu[ Region[{tag}] ] = 1.0 / ({mu_r_magnet} * mu0);")
            
            # Magnetization vector for this magnet (radial direction with polarity)
            # For surface-mounted magnets, magnetization points radially outward (N) or inward (S)
            # hc = polarity * Hc * r_hat where r_hat is unit radial vector
            cos_angle = np.cos(angle)
            sin_angle = np.sin(angle)
            hc_x = polarity * magnet_br / mu_0 * cos_angle
            hc_y = polarity * magnet_br / mu_0 * sin_angle
            magnet_hc_assignments.append(f"  hc[ Region[{tag}] ] = Vector[{hc_x:.6e}, {hc_y:.6e}, 0];")
        
        magnets_region_str = ", ".join(magnet_region_list) if magnet_region_list else "0"
        
        # Check if we have windings (slots) - they're optional
        has_windings = "Windings" if len(magnet_regions) > 0 else ""  # Simplified check
        
        getdp_problem = f"""
/* GetDP Magnetostatic Problem for Permanent Magnet Motor */
/* Each magnet pole has individual magnetization direction */

Group {{
  Shaft = Region[1];
  Rotor = Region[2];
  Airgap = Region[4];
  Stator = Region[5];
  Windings = Region[6];
  OuterBoundary = Region[200];
  
  // Individual magnet regions with polarities
  Magnets = Region[{{{magnets_region_str}}}];
  
  Vol_Mag = Region[{{Shaft, Rotor, Magnets, Airgap, Stator}}];
  Vol_NonMag = Region[{{Shaft, Airgap}}];
  Vol_Steel = Region[{{Rotor, Stator}}];
}}

Function {{
  mu0 = {mu_0};
  
  // Reluctivity nu = 1/(mu0 * mu_r) by region
  nu[ Region[{{Shaft, Airgap}}] ] = 1.0 / mu0;
  nu[ Region[{{Rotor, Stator}}] ] = 1.0 / ({mu_r_steel} * mu0);
  
  // Reluctivity for each magnet
{chr(10).join(magnet_nu_assignments)}
  
  // Coercive field vector for each magnet (radial with polarity)
{chr(10).join(magnet_hc_assignments)}
}}

Jacobian {{
  {{ Name Vol; Case {{ {{ Region All; Jacobian Vol; }} }} }}
  {{ Name Sur; Case {{ {{ Region All; Jacobian Sur; }} }} }}
}}

Integration {{
  {{ Name I1; Case {{ {{ Type Gauss; Case {{ {{ GeoElement Triangle; NumberOfPoints 4; }} }} }} }} }}
}}

Constraint {{
  {{ Name a;
    Type Assign;
    Case {{
      {{ Region OuterBoundary; Value 0.0; }}
    }}
  }}
}}

FunctionSpace {{
  {{ Name Hcurl_a; Type Form1P;
    BasisFunction {{
      {{ Name se; NameOfCoef ae; Function BF_PerpendicularEdge; Support Vol_Mag; Entity NodesOf[All]; }}
    }}
    Constraint {{
      {{ NameOfCoef ae; EntityType NodesOf; NameOfConstraint a; }}
    }}
  }}
}}

Formulation {{
  {{ Name MagSta_a; Type FemEquation;
    Quantity {{
      {{ Name a; Type Local; NameOfSpace Hcurl_a; }}
    }}
    Equation {{
      Galerkin {{ [ nu[] * Dof{{d a}}, {{d a}} ];
        In Vol_Mag; Jacobian Vol; Integration I1; }}
      
      Galerkin {{ [ hc[], {{d a}} ];
        In Magnets; Jacobian Vol; Integration I1; }}
    }}
  }}
}}

Resolution {{
  {{ Name MagSta;
    System {{
      {{ Name A; NameOfFormulation MagSta_a; }}
    }}
    Operation {{
      Generate[A]; Solve[A]; SaveSolution[A];
    }}
  }}
}}

PostProcessing {{
  {{ Name MagSta_a; NameOfFormulation MagSta_a;
    Quantity {{
      {{ Name az; Value {{ Term {{ [ CompZ[{{a}}] ]; In Vol_Mag; Jacobian Vol; }} }} }}
      {{ Name b; Value {{ Term {{ [ {{d a}} ]; In Vol_Mag; Jacobian Vol; }} }} }}
    }}
  }}
}}

PostOperation {{
  {{ Name MagSta_a; NameOfPostProcessing MagSta_a;
    Operation {{
      Print[ az, OnElementsOf Vol_Mag, File "res/az.pos", SendToServer "Output/az [Wb/m]" ];
      Print[ b, OnElementsOf Vol_Mag, File "res/b.pos", SendToServer "Output/b [T]" ];
    }}
  }}
}}
"""
        
        with open(pro_file, 'w') as f:
            f.write(getdp_problem)
    
    def _parse_getdp_results(self, temp_dir: str, mesh_data: Dict) -> Dict:
        """Parse GetDP results and return solution dictionary"""
        import re
        import os
        
        # Read result files
        az_file = os.path.join(temp_dir, "res", "az.pos")
        b_file = os.path.join(temp_dir, "res", "b.pos")
        
        nodes = mesh_data['nodes']
        elements = mesh_data['elements']
        num_nodes = len(nodes)
        num_elements = len(elements)
        
        # Parse vector potential (z-component) - output is on elements
        A = np.zeros(num_elements)
        if os.path.exists(az_file):
            A = self._parse_pos_file_scalar(az_file, num_elements)
        
        # Parse flux density vector (Bx, By) - output is on elements  
        B_elements = np.zeros((num_elements, 2))
        if os.path.exists(b_file):
            B_elements = self._parse_pos_file_vector(b_file, num_elements)
            B_mag = np.linalg.norm(B_elements, axis=1)
            nonzero_count = np.sum(B_mag > 1e-10)
            print(f"Parsed B: {nonzero_count}/{len(B_elements)} elements non-zero, range [{np.min(B_mag):.3f}, {np.max(B_mag):.3f}] T")
        else:
            print(f"Warning: GetDP flux density file not found: {b_file}")
        
        # Interpolate B to nodes
        B_nodes = np.zeros((num_nodes, 2))
        node_count = np.zeros(num_nodes)
        
        for elem_idx, elem in enumerate(elements):
            for node_idx in elem.astype(int):
                B_nodes[node_idx] += B_elements[elem_idx]
                node_count[node_idx] += 1
        
        for i in range(num_nodes):
            if node_count[i] > 0:
                B_nodes[i] /= node_count[i]
        
        return {
            'vector_potential': A,
            'flux_density_nodes': B_nodes,
            'flux_density_elements': B_elements,
            'node_positions': mesh_data['nodes'],
            'mesh': mesh_data,
            'rotor_angle': 0.0  # GetDP results are static
        }
    
    def _parse_pos_file_scalar(self, filename: str, expected_size: int) -> np.ndarray:
        """Parse GetDP .pos file format for scalar values"""
        import re
        values = []
        
        with open(filename, 'r') as f:
            content = f.read()
            
            # Extract scalar values from pos file
            # Format: ST(x,y,z){value};
            pattern = r'ST\([^)]+\)\s*\{\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)'
            matches = re.findall(pattern, content)
            
            for match in matches:
                values.append(float(match))
        
        result = np.array(values)
        
        # Pad or truncate to expected size
        if len(result) < expected_size:
            result = np.pad(result, (0, expected_size - len(result)))
        elif len(result) > expected_size:
            result = result[:expected_size]
        
        return result
    
    def _parse_pos_file_vector(self, filename: str, expected_size: int) -> np.ndarray:
        """Parse GetDP .pos file format for vector values (2D: Bx, By)"""
        import re
        values = []
        
        with open(filename, 'r') as f:
            content = f.read()
            
            # Extract vector values from pos file
            # Format: VT(x,y,z){vx,vy,vz};
            pattern = r'VT\([^)]+\)\s*\{\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)\s*,\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)'
            matches = re.findall(pattern, content)
            
            for match in matches:
                values.append([float(match[0]), float(match[1])])
        
        result = np.array(values)
        
        # Handle empty or wrong-sized results
        if len(result) == 0:
            result = np.zeros((expected_size, 2))
        elif len(result) < expected_size:
            padding = np.zeros((expected_size - len(result), 2))
            result = np.vstack([result, padding])
        elif len(result) > expected_size:
            result = result[:expected_size]
        
        return result
    
    def _solve_analytical_fallback(self, mesh_data: Dict, rotor_angle: float = 0.0) -> Dict:
        """
        Fallback analytical solution if scikit-fem is not available
        Uses simplified field approximation based on geometry
        """
        nodes = mesh_data['nodes']
        elements = mesh_data['elements']
        num_nodes = mesh_data['num_nodes']
        
        # Calculate magnetic vector potential (A) at each node (simplified)
        A = np.zeros(num_nodes)
        B_field = np.zeros((num_nodes, 2))
        
        for i, node in enumerate(nodes):
            x, y = node
            r = np.sqrt(x**2 + y**2)
            theta = np.arctan2(y, x)
            
            # Simplified analytical field
            r_rotor = self.params.rotor_outer_diameter / 2
            r_magnet = r_rotor + self.params.magnet_thickness
            r_stator_bore = self.params.stator_inner_diameter / 2
            
            if r < r_rotor:
                B_mag = 0.1  # Low field in rotor core
            elif r < r_magnet:
                # In magnet - strong radial field
                pole_angle = 2 * np.pi / int(self.params.num_poles)
                pole_num = int((theta + rotor_angle * np.pi / 180) / pole_angle) % int(self.params.num_poles)
                polarity = 1 if pole_num % 2 == 0 else -1
                B_mag = polarity * self.params.magnet_br
            elif r < r_stator_bore:
                # Airgap
                angle_offset = theta + rotor_angle * np.pi / 180
                B_mag = self.params.magnet_br * np.cos(int(self.params.num_poles/2) * angle_offset)
            else:
                # Stator
                angle_offset = theta + rotor_angle * np.pi / 180
                B_mag = 0.5 * self.params.magnet_br * np.cos(int(self.params.num_poles/2) * angle_offset)
            
            # Convert to Cartesian
            B_r = B_mag
            B_theta = 0.0
            
            B_field[i, 0] = B_r * np.cos(theta) - B_theta * np.sin(theta)
            B_field[i, 1] = B_r * np.sin(theta) + B_theta * np.cos(theta)
            A[i] = B_mag * r / 2
        
        # Element-wise flux
        element_B = np.zeros((len(elements), 2))
        for i, elem in enumerate(elements):
            element_B[i] = np.mean(B_field[elem.astype(int)], axis=0)
        
        return {
            'vector_potential': A,
            'flux_density_nodes': B_field,
            'flux_density_elements': element_B,
            'mesh': mesh_data,
            'rotor_angle': rotor_angle
        }
    
    def calculate_performance_fea(self, solution: Dict) -> Dict:
        """
        Calculate motor performance metrics from FEA solution
        
        Args:
            solution: FEA solution from solve_magnetostatic()
            
        Returns:
            Performance metrics dictionary
        """
        mesh_data = solution['mesh']
        B_elements = solution['flux_density_elements']
        elements = mesh_data['elements']
        nodes = mesh_data['nodes']
        
        # Calculate torque using Maxwell stress tensor in airgap
        # Simplified calculation
        r_airgap = (self.params.rotor_outer_diameter / 2 + 
                   self.params.magnet_thickness + 
                   self.params.airgap_length / 2)
        
        # Find elements in airgap region
        airgap_elements = []
        for i, elem in enumerate(elements):
            elem_nodes = nodes[elem.astype(int)]
            elem_r = np.mean(np.sqrt(elem_nodes[:, 0]**2 + elem_nodes[:, 1]**2))
            
            r_magnet_outer = self.params.rotor_outer_diameter / 2 + self.params.magnet_thickness
            r_stator_bore = self.params.stator_inner_diameter / 2
            
            if r_magnet_outer < elem_r < r_stator_bore:
                airgap_elements.append(i)
        
        # Calculate average flux density in airgap
        if airgap_elements:
            B_airgap = np.mean(np.linalg.norm(B_elements[airgap_elements], axis=1))
        else:
            B_airgap = 0.5  # Default estimate
        
        # Torque calculation using simplified formula
        # T = (B^2 * r^2 * L * V) / (2 * mu_0)
        mu_0 = 4 * np.pi * 1e-7
        volume = (np.pi * r_airgap**2 * self.params.stack_length * 1e-9)  # m^3
        torque = (B_airgap**2 * r_airgap * 1e-3 * self.params.stack_length * 1e-3 * 
                 2 * np.pi * r_airgap * 1e-3) / (2 * mu_0)
        
        # Power calculation
        omega = 2 * np.pi * self.params.rated_speed / 60  # rad/s
        power = torque * omega / 1000  # kW
        
        # Back EMF estimation
        flux_per_pole = B_airgap * (2 * np.pi * r_airgap * 1e-3) * (self.params.stack_length * 1e-3) / int(self.params.num_poles)
        back_emf = 2 * np.pi * self.params.rated_speed / 60 * flux_per_pole * self.params.turns_per_coil * 3 / np.sqrt(2)
        
        return {
            'torque': torque,
            'power': power,
            'back_emf': back_emf,
            'airgap_flux_density': B_airgap,
            'peak_flux_density': np.max(np.linalg.norm(B_elements, axis=1)),
            'solver_type': 'FEA',
            'mesh_nodes': mesh_data['num_nodes'],
            'mesh_elements': mesh_data['num_elements']
        }
    
    def get_airgap_flux(self, solution: Dict, n_points: int = 360, smooth: bool = False) -> np.ndarray:
        """
        Extract airgap flux at specific circumferential positions
        
        Args:
            solution: FEA solution from solve_magnetostatic()
            n_points: Number of circumferential points to sample
            smooth: Apply smoothing filter to reduce numerical noise (deprecated - not needed with proper meshing)
            
        Returns:
            Array of radial flux density (Tesla) at each circumferential position
        """
        mesh_data = solution['mesh']
        B_nodes = solution['flux_density_nodes']
        nodes = mesh_data['nodes']
        
        # CONVERT FROM MM TO METERS (mesh coordinates are in SI units)
        mm_to_m = 0.001
        
        # Calculate airgap sampling radius (midpoint between magnet outer and stator bore)
        r_magnet_outer = (self.params.rotor_outer_diameter / 2 + self.params.magnet_thickness) * mm_to_m
        r_stator_bore = self.params.stator_inner_diameter / 2 * mm_to_m
        r_airgap = (r_magnet_outer + r_stator_bore) / 2
        
        # With our 3-layer airgap mesh, we have nodes on 4 circles:
        # - r_inner (magnet_outer)
        # - r_mid1 (magnet_outer + gap/3)
        # - r_mid2 (magnet_outer + 2*gap/3)  <- closest to sampling radius
        # - r_outer (stator_bore)
        
        # Find nodes in the airgap region
        node_radii = np.sqrt(nodes[:, 0]**2 + nodes[:, 1]**2)
        airgap_tolerance = 0.5 * mm_to_m  # 0.5mm tolerance in meters
        airgap_mask = (node_radii >= r_magnet_outer - airgap_tolerance) & (node_radii <= r_stator_bore + airgap_tolerance)
        
        if not np.any(airgap_mask):
            print(f"Warning: No airgap nodes found in range {r_magnet_outer*1000:.1f}-{r_stator_bore*1000:.1f}mm")
            return np.zeros(n_points)
        
        airgap_nodes = nodes[airgap_mask]
        airgap_B = B_nodes[airgap_mask]
        airgap_radii = node_radii[airgap_mask]
        
        # Calculate angles of airgap nodes
        airgap_angles = np.arctan2(airgap_nodes[:, 1], airgap_nodes[:, 0])
        airgap_angles[airgap_angles < 0] += 2 * np.pi  # Convert to [0, 2π]
        
        # Sort by angle
        sort_indices = np.argsort(airgap_angles)
        sorted_angles = airgap_angles[sort_indices]
        sorted_nodes = airgap_nodes[sort_indices]
        sorted_B = airgap_B[sort_indices]
        sorted_radii = airgap_radii[sort_indices]
        
        # For each angle bin, select the node with MAXIMUM radial flux magnitude
        # This captures the peak flux regardless of radial position
        angle_bins = np.linspace(0, 2*np.pi, n_points + 1)
        B_radial_sampled = []
        sampled_angles = []
        
        # Debug: Track selected nodes
        selected_node_radii = []
        selected_node_B_mag = []
        
        for i in range(n_points):
            angle_min = angle_bins[i]
            angle_max = angle_bins[i+1]
            bin_mask = (sorted_angles >= angle_min) & (sorted_angles < angle_max)
            
            if np.any(bin_mask):
                # Get nodes in this angular bin
                bin_radii = sorted_radii[bin_mask]
                bin_nodes = sorted_nodes[bin_mask]
                bin_B = sorted_B[bin_mask]
                bin_angles = sorted_angles[bin_mask]
                
                # Calculate radial component for ALL nodes in this bin
                B_radial_all = []
                for j, (node, B) in enumerate(zip(bin_nodes, bin_B)):
                    x, y = node
                    r = bin_radii[j]
                    Bx, By = B
                    B_r = (Bx * x + By * y) / r if r > 0 else 0
                    B_radial_all.append(abs(B_r))  # Use magnitude for comparison
                
                # Select node with MAXIMUM radial flux magnitude
                max_idx = np.argmax(B_radial_all)
                node = bin_nodes[max_idx]
                B = bin_B[max_idx]
                
                # Calculate signed radial component for selected node
                x, y = node
                r = bin_radii[max_idx]
                Bx, By = B
                B_radial = (Bx * x + By * y) / r if r > 0 else 0
                
                B_radial_sampled.append(B_radial)
                sampled_angles.append(bin_angles[max_idx])
                
                # Debug
                selected_node_radii.append(r)
                selected_node_B_mag.append(np.sqrt(Bx**2 + By**2))
        
        if len(B_radial_sampled) < n_points:
            # Some bins empty - use simple average interpolation for all angles
            print(f"Warning: Only {len(B_radial_sampled)}/{n_points} angle bins populated, using interpolation")
            if len(B_radial_sampled) == 0:
                return np.zeros(n_points)
            
            # Interpolate with wrap-around
            sampled_angles_arr = np.array(sampled_angles)
            B_radial_sampled_arr = np.array(B_radial_sampled)
            
            # Extend for wrap-around interpolation
            angles_extended = np.concatenate([sampled_angles_arr - 2*np.pi, sampled_angles_arr, sampled_angles_arr + 2*np.pi])
            B_extended = np.tile(B_radial_sampled_arr, 3)
            
            target_angles = np.linspace(0, 2*np.pi, n_points, endpoint=False)
            airgap_flux = np.interp(target_angles, angles_extended, B_extended)
        else:
            airgap_flux = np.array(B_radial_sampled)
        
        return airgap_flux
    
    def run_analysis(self, rotor_angle: float = 0.0, mesh_size: float = None) -> Tuple[Dict, Dict]:
        """
        Complete FEA workflow: mesh generation, solve, and calculate performance
        
        Args:
            rotor_angle: Rotor angle in degrees
            mesh_size: Mesh characteristic size in mm
            
        Returns:
            Tuple of (solution, performance_metrics)
        """
        # Create mesh
        mesh_data = self.create_mesh(mesh_size=mesh_size)
        
        # Solve magnetostatic problem
        solution = self.solve_magnetostatic(mesh_data, rotor_angle)
        
        # Calculate performance
        performance = self.calculate_performance_fea(solution)
        
        return solution, performance
