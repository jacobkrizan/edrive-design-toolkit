"""
Magnetic Equivalent Circuit (MEC) Solver
Computes flux distribution using geometry-based magnetic network analysis
"""

import numpy as np
from scipy.sparse import lil_matrix, csr_matrix
from scipy.sparse.linalg import spsolve
from typing import Dict, List, Tuple
from motor_parameters import MotorParameters, PerformanceMetrics
from geometry import MotorGeometry


class MagneticCircuitSolver:
    """Solves magnetic equivalent circuit for flux density distribution"""
    
    def __init__(self, params: MotorParameters, geometry: MotorGeometry):
        self.params = params
        self.geometry = geometry
        self.mu0 = 4 * np.pi * 1e-7  # H/m
        
        # Build MEC network
        self.nodes = []
        self.elements = []
        self.node_count = 0
        
    def build_network(self):
        """Build magnetic equivalent circuit network"""
        # Create nodes for each region
        self.node_count = 0
        
        # Airgap nodes (one per mechanical degree or coarser)
        n_airgap = 360  # nodes around airgap
        self.airgap_nodes = list(range(self.node_count, self.node_count + n_airgap))
        self.node_count += n_airgap
        
        # Magnet nodes (TWO per pole: top and bottom for parallel MMF/reluctance)
        num_poles = int(self.params.num_poles)
        n_magnet_nodes = num_poles * 2  # Top and bottom for each magnet
        self.magnet_nodes = list(range(self.node_count, self.node_count + n_magnet_nodes))
        self.node_count += n_magnet_nodes
        
        # Tooth nodes
        num_slots = int(self.params.num_slots)
        self.tooth_nodes = list(range(self.node_count, self.node_count + num_slots))
        self.node_count += num_slots
        
        # Yoke nodes (stator back iron)
        n_yoke = num_slots
        self.yoke_nodes = list(range(self.node_count, self.node_count + n_yoke))
        self.node_count += n_yoke
        
        # Rotor core nodes
        self.rotor_nodes = list(range(self.node_count, self.node_count + num_poles))
        self.node_count += num_poles
        
        # Reference node (ground)
        self.ground_node = self.node_count
        self.node_count += 1
        
    def calculate_reluctances(self) -> Dict:
        """Calculate reluctances for all MEC elements"""
        reluctances = {}
        
        # Airgap reluctance
        airgap_area = self.params.stack_length * self.params.pole_pitch
        airgap_length = self.params.airgap_length * 1e-3  # Convert to meters
        reluctances['airgap'] = airgap_length / (self.mu0 * airgap_area * 1e-6)
        
        # Magnet reluctance
        magnet_area = self.params.stack_length * self.params.pole_pitch * self.params.magnet_arc_ratio
        magnet_thickness = self.params.magnet_thickness * 1e-3
        reluctances['magnet'] = magnet_thickness / (self.mu0 * self.params.magnet_mu_r * magnet_area * 1e-6)
        
        # Tooth reluctance
        tooth_area = self.params.tooth_width * self.params.stack_length
        tooth_length = self.params.slot_depth * 1e-3
        reluctances['tooth'] = tooth_length / (self.mu0 * self.params.steel_mu_r * tooth_area * 1e-6)
        
        # Yoke reluctance
        yoke_thickness = (self.params.stator_outer_diameter - self.params.stator_inner_diameter) / 2 - self.params.slot_depth
        yoke_area = yoke_thickness * self.params.stack_length
        yoke_length = self.params.slot_pitch * 1e-3
        reluctances['yoke'] = yoke_length / (self.mu0 * self.params.steel_mu_r * yoke_area * 1e-6)
        
        # Rotor core reluctance
        rotor_area = (self.params.rotor_outer_diameter - self.params.rotor_inner_diameter) / 2 * self.params.stack_length
        rotor_length = self.params.pole_pitch / 2 * 1e-3
        reluctances['rotor'] = rotor_length / (self.mu0 * self.params.steel_mu_r * rotor_area * 1e-6)
        
        return reluctances
    
    def solve_mec(self, rotor_angle: float = 0.0) -> Dict:
        """Solve magnetic equivalent circuit using nodal analysis for given rotor position"""
        self.build_network()
        reluctances = self.calculate_reluctances()
        
        # Build conductance matrix (inverse of reluctance) and MMF source vector
        # Using modified nodal analysis: [G]{Φ} = {MMF}
        # where G is the permeance matrix, Φ is the vector of magnetic potentials
        
        n_poles = int(self.params.num_poles)
        n_slots = int(self.params.num_slots)
        n_airgap = len(self.airgap_nodes)
        
        # Initialize system matrix and RHS vector
        n_nodes = self.node_count
        G = lil_matrix((n_nodes, n_nodes))  # Conductance (permeance) matrix
        MMF = np.zeros(n_nodes)  # MMF source vector
        
        # Calculate permeances (inverse reluctances)
        P_airgap = 1.0 / reluctances['airgap']
        P_magnet = 1.0 / reluctances['magnet']
        P_tooth = 1.0 / reluctances['tooth']
        P_yoke = 1.0 / reluctances['yoke']
        P_rotor = 1.0 / reluctances['rotor']
        
        # Magnet MMF
        magnet_mmf = self.params.magnet_br * self.params.magnet_thickness * 1e-3 / (self.mu0 * self.params.magnet_mu_r)
        
        # Winding MMF (simplified - actual implementation would use winding function)
        winding_config = self.geometry.get_winding_positions()
        # Calculate MMF per phase based on current and turns
        turns_per_coil = 10  # Simplified - should be calculated from design
        winding_mmf = self.params.rated_current * turns_per_coil
        
        # Build circuit connections
        # Circuit topology (top to bottom):
        # 1. Stator yoke (yoke nodes connected circumferentially)
        # 2. Tooth + Winding MMF (tooth nodes)
        # 3. Airgap (airgap nodes)
        # 4. Magnet (magnet MMF in parallel with magnet reluctance)
        # 5. Rotor backiron (rotor nodes connected circumferentially)
        
        # Connect yoke segments circumferentially (stator backiron at top)
        for i in range(len(self.yoke_nodes)):
            yoke_node1 = self.yoke_nodes[i]
            yoke_node2 = self.yoke_nodes[(i + 1) % len(self.yoke_nodes)]
            
            G[yoke_node1, yoke_node1] += P_yoke
            G[yoke_node1, yoke_node2] -= P_yoke
            G[yoke_node2, yoke_node1] -= P_yoke
            G[yoke_node2, yoke_node2] += P_yoke
        
        # Connect yoke to teeth (series connection)
        for i in range(n_slots):
            yoke_node = self.yoke_nodes[i]
            tooth_node = self.tooth_nodes[i]
            
            # Direct connection (negligible reluctance)
            P_connection = P_tooth * 1000
            
            G[yoke_node, yoke_node] += P_connection
            G[yoke_node, tooth_node] -= P_connection
            G[tooth_node, yoke_node] -= P_connection
            G[tooth_node, tooth_node] += P_connection
            
            # Add winding MMF to tooth node (in series with tooth reluctance)
            # Determine which phase and polarity for this slot
            slot_mmf = 0.0
            for phase, windings in winding_config.items():
                for winding in windings:
                    if winding['slot_index'] == i:
                        # Winding MMF depends on current phase angle
                        # For now, simplified as DC current
                        slot_mmf = winding['polarity'] * winding_mmf
                        break
            
            # Apply winding MMF (acts as voltage source in series with tooth)
            if slot_mmf != 0:
                MMF[tooth_node] += slot_mmf * P_connection
                MMF[yoke_node] -= slot_mmf * P_connection
        
        # Connect teeth to airgap (tooth reluctance)
        for slot_idx in range(n_slots):
            tooth_node = self.tooth_nodes[slot_idx]
            slot_angle = 2 * np.pi * slot_idx / n_slots
            
            # Find nearby airgap node
            ag_idx_center = int((slot_angle % (2 * np.pi)) / (2 * np.pi) * n_airgap)
            airgap_node = self.airgap_nodes[ag_idx_center % n_airgap]
            
            G[tooth_node, tooth_node] += P_tooth
            G[tooth_node, airgap_node] -= P_tooth
            G[airgap_node, tooth_node] -= P_tooth
            G[airgap_node, airgap_node] += P_tooth
        
        # Connect airgap to magnets (airgap reluctance)
        # Distribute airgap connections across magnet pole arcs
        for pole_idx in range(n_poles):
            magnet_top_idx = 2 * pole_idx  # Top node (connects to airgap)
            magnet_top_node = self.magnet_nodes[magnet_top_idx]
            
            # Determine which airgap nodes this pole influences
            pole_angle = 2 * np.pi * pole_idx / n_poles - rotor_angle
            magnet_arc = (2 * np.pi / n_poles) * self.params.magnet_arc_ratio
            
            # Connect magnet top to airgap points under this pole
            for ag_idx in range(n_airgap):
                theta = 2 * np.pi * ag_idx / n_airgap
                angular_dist = abs(((theta - pole_angle + np.pi) % (2 * np.pi)) - np.pi)
                
                if angular_dist < magnet_arc / 2:
                    # This airgap point is under this magnet
                    airgap_node = self.airgap_nodes[ag_idx]
                    
                    # Weight based on position under magnet
                    weight = np.cos(np.pi * angular_dist / magnet_arc)
                    P_ag_weighted = P_airgap * weight * (n_airgap / (n_poles * self.params.magnet_arc_ratio))
                    
                    G[magnet_top_node, magnet_top_node] += P_ag_weighted
                    G[magnet_top_node, airgap_node] -= P_ag_weighted
                    G[airgap_node, magnet_top_node] -= P_ag_weighted
                    G[airgap_node, airgap_node] += P_ag_weighted
        
        # Magnet: MMF source in parallel with magnet reluctance
        # Connect magnet top to magnet bottom through reluctance and MMF
        for pole_idx in range(n_poles):
            magnet_top_idx = 2 * pole_idx  # Top node (connects to airgap)
            magnet_bottom_idx = 2 * pole_idx + 1  # Bottom node (connects to rotor)
            magnet_top_node = self.magnet_nodes[magnet_top_idx]
            magnet_bottom_node = self.magnet_nodes[magnet_bottom_idx]
            
            # Magnet polarity (alternates N, S, N, S...)
            polarity = 1 if pole_idx % 2 == 0 else -1
            
            # Magnet reluctance connects top to bottom
            G[magnet_top_node, magnet_top_node] += P_magnet
            G[magnet_top_node, magnet_bottom_node] -= P_magnet
            G[magnet_bottom_node, magnet_top_node] -= P_magnet
            G[magnet_bottom_node, magnet_bottom_node] += P_magnet
            
            # Add magnet MMF source (in parallel with reluctance)
            MMF[magnet_top_node] += polarity * magnet_mmf * P_magnet
            MMF[magnet_bottom_node] -= polarity * magnet_mmf * P_magnet
        
        # Connect magnet bottom to rotor backiron
        for pole_idx in range(n_poles):
            magnet_bottom_idx = 2 * pole_idx + 1  # Bottom node
            magnet_bottom_node = self.magnet_nodes[magnet_bottom_idx]
            rotor_node = self.rotor_nodes[pole_idx]
            
            # Direct connection (negligible reluctance)
            P_connection = P_magnet * 1000
            
            G[magnet_bottom_node, magnet_bottom_node] += P_connection
            G[magnet_bottom_node, rotor_node] -= P_connection
            G[rotor_node, magnet_bottom_node] -= P_connection
            G[rotor_node, rotor_node] += P_connection
        
        # Connect rotor segments circumferentially (rotor backiron at bottom)
        for i in range(n_poles):
            rotor_node1 = self.rotor_nodes[i]
            rotor_node2 = self.rotor_nodes[(i + 1) % n_poles]
            
            G[rotor_node1, rotor_node1] += P_rotor
            G[rotor_node1, rotor_node2] -= P_rotor
            G[rotor_node2, rotor_node1] -= P_rotor
            G[rotor_node2, rotor_node2] += P_rotor
        
        # Ground one node (reference potential = 0)
        # Ground the first yoke node to set reference potential
        ground_idx = self.yoke_nodes[0]
        G[ground_idx, :] = 0
        G[ground_idx, ground_idx] = 1
        MMF[ground_idx] = 0
        
        # Solve system: G * Φ = MMF
        G_csr = G.tocsr()
        
        # Use least squares solver since the matrix may be rank-deficient
        # (balanced MMF sources can create a floating potential)
        from scipy.sparse.linalg import lsqr
        result = lsqr(G_csr, MMF, atol=1e-10, btol=1e-10)
        potentials = result[0]
        
        # Calculate fluxes from potentials
        fluxes = self._calculate_fluxes_from_potentials(potentials, reluctances, rotor_angle)
        
        return {
            'potentials': potentials,
            'fluxes': fluxes,
            'reluctances': reluctances
        }
    
    def _calculate_fluxes_from_potentials(self, potentials: np.ndarray, reluctances: Dict, rotor_angle: float) -> Dict:
        """Calculate flux values from solved potentials"""
        fluxes = {
            'airgap': [],
            'teeth': [],
            'yoke': [],
            'magnets': [],
            'rotor': []
        }
        
        n_poles = int(self.params.num_poles)
        n_slots = int(self.params.num_slots)
        n_airgap = len(self.airgap_nodes)
        
        # Airgap fluxes - calculate from potential difference
        for ag_idx in range(n_airgap):
            airgap_node = self.airgap_nodes[ag_idx]
            theta = 2 * np.pi * ag_idx / n_airgap
            
            # Find nearest magnet top node
            pole_angle = theta + rotor_angle
            pole_idx = int((pole_angle % (2 * np.pi)) / (2 * np.pi) * n_poles)
            magnet_top_idx = 2 * (pole_idx % n_poles)  # Top node index
            magnet_top_node = self.magnet_nodes[magnet_top_idx]
            
            flux = (potentials[magnet_top_node] - potentials[airgap_node]) / reluctances['airgap']
            fluxes['airgap'].append(flux)
        
        # Tooth fluxes
        for slot_idx in range(n_slots):
            tooth_node = self.tooth_nodes[slot_idx]
            slot_angle = 2 * np.pi * slot_idx / n_slots
            ag_idx = int((slot_angle % (2 * np.pi)) / (2 * np.pi) * n_airgap)
            airgap_node = self.airgap_nodes[ag_idx]
            
            flux = (potentials[airgap_node] - potentials[tooth_node]) / reluctances['tooth']
            fluxes['teeth'].append(flux)
        
        # Yoke fluxes (flux between adjacent yoke segments)
        for i in range(len(self.yoke_nodes)):
            yoke_node1 = self.yoke_nodes[i]
            yoke_node2 = self.yoke_nodes[(i + 1) % len(self.yoke_nodes)]
            
            flux = (potentials[yoke_node1] - potentials[yoke_node2]) / reluctances['yoke']
            fluxes['yoke'].append(flux)
        
        # Magnet fluxes (top to bottom through reluctance)
        for pole_idx in range(n_poles):
            magnet_top_idx = 2 * pole_idx
            magnet_bottom_idx = 2 * pole_idx + 1
            magnet_top_node = self.magnet_nodes[magnet_top_idx]
            magnet_bottom_node = self.magnet_nodes[magnet_bottom_idx]
            
            flux = (potentials[magnet_top_node] - potentials[magnet_bottom_node]) / reluctances['magnet']
            fluxes['magnets'].append(flux)
        
        # Rotor backiron fluxes (flux between adjacent rotor segments)
        for i in range(n_poles):
            rotor_node1 = self.rotor_nodes[i]
            rotor_node2 = self.rotor_nodes[(i + 1) % n_poles]
            
            flux = (potentials[rotor_node1] - potentials[rotor_node2]) / reluctances['rotor']
            fluxes['rotor'].append(flux)
        
        return fluxes
    
    def _calculate_fluxes(self, potentials: np.ndarray, reluctances: Dict) -> Dict:
        """Calculate flux values from potentials"""
        fluxes = {
            'airgap': [],
            'teeth': [],
            'yoke': [],
            'magnets': []
        }
        
        # Airgap flux density
        for i in range(len(self.airgap_nodes)):
            ag_node = self.airgap_nodes[i]
            # Find corresponding tooth
            slot_idx = int(i * self.params.num_slots / len(self.airgap_nodes))
            tooth_node = self.tooth_nodes[slot_idx]
            
            flux = (potentials[ag_node] - potentials[tooth_node]) / reluctances['airgap']
            fluxes['airgap'].append(flux)
        
        # Tooth flux
        for i, tooth_node in enumerate(self.tooth_nodes):
            yoke_node = self.yoke_nodes[i]
            flux = (potentials[tooth_node] - potentials[yoke_node]) / reluctances['tooth']
            fluxes['teeth'].append(flux)
        
        # Magnet flux
        for i, rotor_node in enumerate(self.rotor_nodes):
            airgap_idx = int(i * len(self.airgap_nodes) / len(self.rotor_nodes))
            ag_node = self.airgap_nodes[airgap_idx]
            flux = (potentials[rotor_node] - potentials[ag_node]) / reluctances['magnet']
            fluxes['magnets'].append(flux)
        
        return fluxes
    
    def calculate_performance(self, rotor_angle: float = 0.0) -> PerformanceMetrics:
        """Calculate all performance metrics"""
        metrics = PerformanceMetrics()
        
        # Solve MEC
        solution = self.solve_mec(rotor_angle)
        fluxes = solution['fluxes']
        
        # Airgap flux density
        airgap_area = self.params.stack_length * self.params.pole_pitch * 1e-6
        airgap_flux_densities = [f / airgap_area for f in fluxes['airgap']]
        metrics.airgap_flux_density_peak = max(airgap_flux_densities)
        
        # Tooth flux density
        tooth_area = self.params.tooth_width * self.params.stack_length * 1e-6
        # Account for multiple teeth carrying flux
        effective_tooth_area = tooth_area * 2  # Flux spreads across adjacent teeth
        tooth_flux_densities = [abs(f) / effective_tooth_area for f in fluxes['teeth']]
        metrics.tooth_flux_density = np.mean(tooth_flux_densities)
        
        # Yoke flux density
        yoke_thickness = (self.params.stator_outer_diameter - self.params.stator_inner_diameter) / 2 - self.params.slot_depth
        yoke_area = yoke_thickness * self.params.stack_length * 1e-6
        metrics.yoke_flux_density = metrics.airgap_flux_density_peak * self.params.pole_pitch / (2 * yoke_area * 1e6)
        
        # Back EMF calculation
        winding_config = self.geometry.get_winding_positions()
        flux_per_pole = np.mean([abs(f) for f in fluxes['magnets']])
        turns_per_phase = sum([w['turns'] for w in winding_config['A']])
        metrics.flux_linkage = flux_per_pole * turns_per_phase / self.params.parallel_paths
        omega = 2 * np.pi * self.params.rated_speed / 60.0
        metrics.back_emf = omega * metrics.flux_linkage * self.params.num_poles / 2 / np.sqrt(2)
        
        # Torque calculation
        torque_angles = np.linspace(0, 2 * np.pi / self.params.num_poles, 36)
        torques = []
        for angle in torque_angles:
            # Simplified torque calculation from flux-current interaction
            T = (3 / 2) * (self.params.num_poles / 2) * metrics.flux_linkage * self.params.rated_current
            torques.append(T)
        metrics.torque = np.mean(torques)
        metrics.torque_ripple = (np.max(torques) - np.min(torques)) / np.mean(torques) * 100
        
        # Power
        metrics.power = metrics.torque * 2 * np.pi * self.params.rated_speed / 60000.0  # kW
        
        # Copper loss
        slot_area = self.params.slot_depth * self.params.tooth_width * self.params.fill_factor
        conductor_area = slot_area / self.params.turns_per_coil
        mean_turn_length = 2 * (self.params.stack_length + np.pi * self.params.stator_inner_diameter / self.params.num_slots)
        phase_resistance = self.params.copper_resistivity * mean_turn_length * 1e-3 * turns_per_phase / (conductor_area * 1e-6) / self.params.parallel_paths
        metrics.winding_resistance = phase_resistance
        metrics.copper_loss = 3 * phase_resistance * self.params.rated_current ** 2
        
        # Iron loss (simplified Steinmetz equation)
        volume_stator = np.pi * ((self.params.stator_outer_diameter / 2) ** 2 - (self.params.stator_inner_diameter / 2) ** 2) * self.params.stack_length * 1e-9
        kh = 100  # Hysteresis loss coefficient
        ke = 0.5  # Eddy current loss coefficient
        metrics.iron_loss = (kh * self.params.electrical_frequency * metrics.tooth_flux_density ** 2 + 
                            ke * self.params.electrical_frequency ** 2 * metrics.tooth_flux_density ** 2) * volume_stator * 7650
        
        # Efficiency
        total_loss = metrics.copper_loss + metrics.iron_loss
        output_power = metrics.power * 1000
        metrics.efficiency = output_power / (output_power + total_loss) * 100
        
        # Power factor (simplified)
        Xd = 2 * np.pi * self.params.electrical_frequency * metrics.flux_linkage / self.params.rated_current
        metrics.power_factor = metrics.back_emf / np.sqrt(metrics.back_emf ** 2 + (Xd * self.params.rated_current) ** 2)
        
        # Current density
        metrics.current_density = self.params.rated_current / (conductor_area * self.params.parallel_paths)
        
        # Temperature rise (simplified thermal model)
        total_surface_area = np.pi * self.params.stator_outer_diameter * self.params.stack_length * 2e-6
        h_conv = 10  # W/m²K convection coefficient
        metrics.temperature_rise = total_loss / (h_conv * total_surface_area)
        
        # Mass and density calculations
        steel_density = 7650  # kg/m³
        magnet_density = 7500  # kg/m³
        copper_density = 8900  # kg/m³
        
        stator_mass = volume_stator * steel_density / 1000
        magnet_volume = self.params.num_poles * self.params.magnet_thickness * self.params.pole_pitch * self.params.magnet_arc_ratio * self.params.stack_length * 1e-9
        magnet_mass = magnet_volume * magnet_density / 1000
        copper_volume = self.params.num_slots * slot_area * mean_turn_length * self.params.fill_factor * 1e-9
        copper_mass = copper_volume * copper_density / 1000
        
        metrics.active_mass = stator_mass + magnet_mass + copper_mass
        metrics.torque_density = metrics.torque / metrics.active_mass
        metrics.power_density = metrics.power / metrics.active_mass
        
        # Cogging torque (simplified)
        metrics.cogging_torque = metrics.torque * 0.02  # Assume 2% of rated torque
        
        return metrics
