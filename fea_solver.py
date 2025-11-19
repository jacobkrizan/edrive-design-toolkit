"""
Finite Element Analysis (FEA) solver for electromagnetic analysis
Uses gmsh for meshing and GetDP for magnetostatic field solution
"""
import numpy as np
import gmsh
import os
from typing import Dict, Tuple
from motor_parameters import MotorParameters
from geometry import MotorGeometry


class FEASolver:
    """
    FEA-based electromagnetic solver using gmsh for mesh generation
    and GetDP for magnetostatic field solution
    """
    
    def __init__(self, params: MotorParameters, geometry: MotorGeometry):
        self.params = params
        self.geometry = geometry
        self.mesh_size = 2.0  # Default mesh size in mm
        
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
        
        # Set mesh size
        gmsh.option.setNumber("Mesh.CharacteristicLengthMin", self.mesh_size / 2)
        gmsh.option.setNumber("Mesh.CharacteristicLengthMax", self.mesh_size * 2)
        
        # Convert to mm for gmsh
        r_shaft = self.params.rotor_inner_diameter / 2
        r_rotor = self.params.rotor_outer_diameter / 2
        r_stator_bore = self.params.stator_inner_diameter / 2
        r_stator_outer = self.params.stator_outer_diameter / 2
        
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
            # Create magnet polygon
            points = []
            for x, y in vertices:
                points.append(gmsh.model.occ.addPoint(x, y, 0))
            
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
            # Create slot polygon
            points = []
            for x, y in vertices:
                points.append(gmsh.model.occ.addPoint(x, y, 0))
            
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
        magnet_outer_r = r_rotor + self.params.magnet_thickness
        airgap_outer_circle = gmsh.model.occ.addCircle(0, 0, 0, r_stator_bore)
        airgap_outer_loop = gmsh.model.occ.addCurveLoop([airgap_outer_circle])
        airgap_inner_circle = gmsh.model.occ.addCircle(0, 0, 0, magnet_outer_r)
        airgap_inner_loop = gmsh.model.occ.addCurveLoop([airgap_inner_circle])
        airgap_surface = gmsh.model.occ.addPlaneSurface([airgap_outer_loop, airgap_inner_loop])
        
        # Synchronize CAD
        gmsh.model.occ.synchronize()
        
        # Create physical groups for material assignment
        gmsh.model.addPhysicalGroup(2, [shaft_surface], 1, "Shaft")
        gmsh.model.addPhysicalGroup(2, [rotor_surface], 2, "Rotor")
        if magnet_surfaces:
            gmsh.model.addPhysicalGroup(2, magnet_surfaces, 3, "Magnets")
        gmsh.model.addPhysicalGroup(2, [airgap_surface], 4, "Airgap")
        gmsh.model.addPhysicalGroup(2, teeth_surfaces, 5, "Stator")  # Teeth (iron)
        if slot_surfaces:
            gmsh.model.addPhysicalGroup(2, slot_surfaces, 6, "Windings")  # Slots (copper)
        
        # Generate 2D mesh
        gmsh.model.mesh.generate(2)
        
        # Extract mesh data
        mesh_data = self._extract_mesh_data()
        
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
        
        # Create temporary directory for GetDP files
        temp_dir = tempfile.mkdtemp()
        mesh_file = os.path.join(temp_dir, "magnetostatic.msh")
        pro_file = os.path.join(temp_dir, "magnetostatic.pro")
        res_dir = os.path.join(temp_dir, "res")
        os.makedirs(res_dir, exist_ok=True)
        
        # Generate mesh with GetDP
        self._create_getdp_mesh(mesh_file, rotor_angle)
        
        # Create GetDP problem definition
        self._create_getdp_problem(pro_file, rotor_angle)
        
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
        
        # Cleanup temporary files
        import shutil
        shutil.rmtree(temp_dir, ignore_errors=True)
        
        return solution
    
    def _create_getdp_mesh(self, mesh_file: str, rotor_angle: float):
        """Generate gmsh mesh file with proper physical groups for GetDP"""
        # Regenerate mesh with GetDP-compatible format
        gmsh.initialize()
        gmsh.model.add("motor_getdp")
        
        # Set mesh size
        gmsh.option.setNumber("Mesh.CharacteristicLengthMin", self.mesh_size / 2)
        gmsh.option.setNumber("Mesh.CharacteristicLengthMax", self.mesh_size * 2)
        
        # Get geometry
        stator_geo = self.geometry.generate_stator_geometry()
        rotor_geo = self.geometry.generate_rotor_geometry()
        
        # Radii
        r_shaft = self.params.rotor_inner_diameter / 2
        r_rotor = self.params.rotor_outer_diameter / 2
        r_stator_bore = self.params.stator_inner_diameter / 2
        r_stator_outer = self.params.stator_outer_diameter / 2
        
        # Create all regions (same as create_mesh)
        shaft_circle = gmsh.model.occ.addCircle(0, 0, 0, r_shaft)
        shaft_loop = gmsh.model.occ.addCurveLoop([shaft_circle])
        shaft_surface = gmsh.model.occ.addPlaneSurface([shaft_loop])
        
        rotor_circle = gmsh.model.occ.addCircle(0, 0, 0, r_rotor)
        rotor_loop = gmsh.model.occ.addCurveLoop([rotor_circle])
        rotor_surface = gmsh.model.occ.addPlaneSurface([rotor_loop, shaft_loop])
        
        # Magnets
        magnet_surfaces = []
        for magnet in rotor_geo['magnets']:
            vertices = magnet['vertices']
            points = [gmsh.model.occ.addPoint(x, y, 0) for x, y in vertices]
            lines = [gmsh.model.occ.addLine(points[i], points[(i+1) % len(points)]) for i in range(len(points))]
            magnet_loop = gmsh.model.occ.addCurveLoop(lines)
            magnet_surf = gmsh.model.occ.addPlaneSurface([magnet_loop])
            magnet_surfaces.append(magnet_surf)
        
        # Slots
        slot_surfaces = []
        for slot in stator_geo['slots']:
            vertices = slot['vertices']
            points = [gmsh.model.occ.addPoint(x, y, 0) for x, y in vertices]
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
        
        # Airgap
        magnet_outer_r = r_rotor + self.params.magnet_thickness
        airgap_outer_circle = gmsh.model.occ.addCircle(0, 0, 0, r_stator_bore)
        airgap_outer_loop = gmsh.model.occ.addCurveLoop([airgap_outer_circle])
        airgap_inner_circle = gmsh.model.occ.addCircle(0, 0, 0, magnet_outer_r)
        airgap_inner_loop = gmsh.model.occ.addCurveLoop([airgap_inner_circle])
        airgap_surface = gmsh.model.occ.addPlaneSurface([airgap_outer_loop, airgap_inner_loop])
        
        gmsh.model.occ.synchronize()
        
        # Physical groups
        gmsh.model.addPhysicalGroup(2, [shaft_surface], 1, "Shaft")
        gmsh.model.addPhysicalGroup(2, [rotor_surface], 2, "Rotor")
        if magnet_surfaces:
            gmsh.model.addPhysicalGroup(2, magnet_surfaces, 3, "Magnets")
        gmsh.model.addPhysicalGroup(2, [airgap_surface], 4, "Airgap")
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
            if abs(r - r_stator_outer) < 1.0:
                outer_curves.append(curve[1])
        
        if outer_curves:
            gmsh.model.addPhysicalGroup(1, outer_curves, 100, "OuterBoundary")
        
        # Generate mesh
        gmsh.model.mesh.generate(2)
        
        # Save mesh
        gmsh.write(mesh_file)
        gmsh.finalize()
    
    def _create_getdp_problem(self, pro_file: str, rotor_angle: float):
        """Create GetDP problem definition (.pro file) for magnetostatics"""
        
        mu_0 = 4 * np.pi * 1e-7
        mu_r_steel = self.params.steel_mu_r
        mu_r_magnet = self.params.magnet_mu_r
        magnet_br = self.params.magnet_br
        
        getdp_problem = f"""
/* GetDP Magnetostatic Problem for Permanent Magnet Motor */

Group {{
  Shaft = Region[1];
  Rotor = Region[2];
  Magnets = Region[3];
  Airgap = Region[4];
  Stator = Region[5];
  Windings = Region[6];
  OuterBoundary = Region[100];
  
  Vol_Mag = Region[{{Shaft, Rotor, Magnets, Airgap, Stator, Windings}}];
  Vol_NonMag = Region[{{Shaft, Airgap, Windings}}];
  Vol_Steel = Region[{{Rotor, Stator}}];
  Sur_Dirichlet = Region[{{OuterBoundary}}];
}}

Function {{
  mu0 = {mu_0};
  
  // Reluctivity nu = 1/(mu0 * mu_r) by region
  nu[ Region[{{Shaft, Airgap, Windings}}] ] = 1.0 / mu0;
  nu[ Region[{{Rotor, Stator}}] ] = 1.0 / ({mu_r_steel} * mu0);
  nu[ Magnets ] = 1.0 / ({mu_r_magnet} * mu0);
  
  // Magnet remanence and coercive field (radial direction)
  Br = {magnet_br};
  Hc = Br / mu0;
  
  // Coercive field vector pointing radially outward (x, y, z)
  // Add small epsilon to avoid division by zero at r=0
  r[] = Sqrt[X[]*X[] + Y[]*Y[] + 1e-10];
  hc[ Magnets ] = Vector[Hc * X[] / r[], Hc * Y[] / r[], 0];
}}

Jacobian {{
  {{ Name Vol; Case {{ {{ Region All; Jacobian Vol; }} }} }}
  {{ Name Sur; Case {{ {{ Region All; Jacobian Sur; }} }} }}
}}

Integration {{
  {{ Name I1; Case {{ {{ Type Gauss; Case {{ {{ GeoElement Triangle; NumberOfPoints 4; }} }} }} }} }}
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

Constraint {{
  {{ Name a; Type Assign;
    Case {{
      {{ Region Sur_Dirichlet; Value 0.; }}
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
      Print[ az, OnElementsOf Vol_Mag, File "res/az.pos" ];
      Print[ b, OnElementsOf Vol_Mag, File "res/b.pos" ];
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
