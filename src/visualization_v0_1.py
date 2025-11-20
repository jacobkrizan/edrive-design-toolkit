"""
Visualization Module
Creates professional 2D renderings of motor cross-section and flux distribution
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, Wedge
from matplotlib.collections import LineCollection
import matplotlib.colors as mcolors
from typing import Dict, List, Tuple
from motor_parameters_v0_1 import MotorParameters, PerformanceMetrics
from geometry_v0_1 import MotorGeometry
from magnetic_circuit_v0_1 import MagneticCircuitSolver


class MotorVisualizer:
    """Handles all visualization of motor geometry and electromagnetic results"""
    
    def __init__(self, params: MotorParameters, geometry: MotorGeometry, solver: MagneticCircuitSolver):
        self.params = params
        self.geometry = geometry
        self.solver = solver
        
        # Color scheme
        self.colors = {
            'stator_steel': '#4A5568',
            'rotor_steel': '#2D3748',
            'magnet_north': '#E53E3E',
            'magnet_south': '#3182CE',
            'winding_A': '#DD6B20',
            'winding_B': '#38A169',
            'winding_C': '#805AD5',
            'airgap': '#EDF2F7',
            'shaft': '#718096'
        }
    
    def create_full_visualization(self, fig, rotor_angle: float = 0.0):
        """Create complete motor visualization with multiple subplots"""
        # Clear figure
        fig.clear()
        
        # Create subplot layout - add MEC diagram
        gs = fig.add_gridspec(2, 3, hspace=0.3, wspace=0.3)
        ax1 = fig.add_subplot(gs[0, :2])  # Motor cross-section (larger)
        ax2 = fig.add_subplot(gs[1, 0])  # Flux density distribution
        ax3 = fig.add_subplot(gs[1, 1])  # Airgap flux density
        ax4 = fig.add_subplot(gs[:, 2])  # MEC diagram (right side)
        
        # Plot motor cross-section
        self.plot_motor_cross_section(ax1, rotor_angle)
        
        # Solve electromagnetic problem
        solution = self.solver.solve_mec(rotor_angle)
        
        # Plot flux density contours
        self.plot_flux_density(ax2, solution, rotor_angle)
        
        # Plot airgap flux density
        self.plot_airgap_flux(ax3, solution)
        
        # Plot MEC diagram
        self.plot_mec_diagram(ax4, solution)
        
        return fig
    
    def plot_motor_cross_section(self, ax, rotor_angle: float = 0.0):
        """Plot detailed 2D cross-section of motor"""
        ax.clear()
        ax.set_aspect('equal')
        ax.set_title('Motor Cross-Section', fontsize=12, fontweight='bold')
        
        # Get geometry
        stator_geo = self.geometry.generate_stator_geometry()
        rotor_geo = self.geometry.generate_rotor_geometry()
        winding_config = self.geometry.get_winding_positions()
        
        num_poles = int(self.params.num_poles)
        
        # Plot stator outer circle
        stator_outer = Circle((0, 0), self.params.stator_outer_diameter / 2, 
                             fill=True, color=self.colors['stator_steel'], 
                             alpha=0.3, linewidth=2, edgecolor='black')
        ax.add_patch(stator_outer)
        
        # Plot stator bore
        stator_bore = Circle((0, 0), self.params.stator_inner_diameter / 2,
                            fill=True, color=self.colors['airgap'],
                            linewidth=1, edgecolor='black')
        ax.add_patch(stator_bore)
        
        # Plot airgap boundary (for reference)
        magnet_outer_r = self.params.rotor_outer_diameter / 2 + self.params.magnet_thickness
        airgap_inner = Circle((0, 0), magnet_outer_r,
                             fill=False, edgecolor='gray', linewidth=0.5, linestyle='--', alpha=0.5)
        ax.add_patch(airgap_inner)
        
        # Plot slots with windings
        for i, slot in enumerate(stator_geo['slots']):
            # Slot body - filled with copper color for winding
            # Determine which phase this slot belongs to
            slot_phase = None
            slot_polarity = 1
            for phase, windings in winding_config.items():
                for winding in windings:
                    if winding['slot_index'] == i:
                        slot_phase = phase
                        slot_polarity = winding['polarity']
                        break
                if slot_phase:
                    break
            
            # Fill slot with copper winding
            copper_color = '#CD7F32' if slot_phase else 'white'  # Copper bronze color
            slot_poly = Polygon(slot['vertices'], fill=True, 
                              color=copper_color, edgecolor='black', linewidth=0.5, alpha=0.7)
            ax.add_patch(slot_poly)
            
            # Add phase marker on top of copper
            if slot_phase:
                center = slot['center']
                phase_color = self.colors[f'winding_{slot_phase}']
                marker = 'o' if slot_polarity > 0 else 'x'
                marker_size = 8
                # Draw colored circle or X to indicate phase and polarity
                ax.plot(center[0], center[1], marker=marker, 
                       color=phase_color, markersize=marker_size, 
                       markeredgewidth=2.5, markeredgecolor=phase_color,
                       markerfacecolor=phase_color if slot_polarity > 0 else 'none')
        
        # Rotate rotor geometry
        cos_a = np.cos(rotor_angle)
        sin_a = np.sin(rotor_angle)
        
        # Plot rotor outer circle
        rotor_outer = Circle((0, 0), self.params.rotor_outer_diameter / 2,
                            fill=True, color=self.colors['rotor_steel'],
                            alpha=0.5, linewidth=2, edgecolor='black')
        ax.add_patch(rotor_outer)
        
        # Plot magnets
        for magnet in rotor_geo['magnets']:
            # Rotate magnet vertices
            vertices = magnet['vertices'].copy()
            rotated_vertices = np.zeros_like(vertices)
            rotated_vertices[:, 0] = vertices[:, 0] * cos_a - vertices[:, 1] * sin_a
            rotated_vertices[:, 1] = vertices[:, 0] * sin_a + vertices[:, 1] * cos_a
            
            # Color based on polarity
            color = self.colors['magnet_north'] if magnet['polarity'] > 0 else self.colors['magnet_south']
            magnet_poly = Polygon(rotated_vertices, fill=True, color=color,
                                 alpha=0.8, edgecolor='black', linewidth=1)
            ax.add_patch(magnet_poly)
            
            # Add N/S label
            center = magnet['center']
            rotated_center = np.array([
                center[0] * cos_a - center[1] * sin_a,
                center[0] * sin_a + center[1] * cos_a
            ])
            label = 'N' if magnet['polarity'] > 0 else 'S'
            ax.text(rotated_center[0], rotated_center[1], label,
                   ha='center', va='center', fontsize=10, fontweight='bold', color='white')
        
        # Plot shaft
        shaft = Circle((0, 0), self.params.rotor_inner_diameter / 2,
                      fill=True, color=self.colors['shaft'],
                      linewidth=1, edgecolor='black')
        ax.add_patch(shaft)
        
        # Formatting
        max_radius = self.params.stator_outer_diameter / 2 * 1.1
        ax.set_xlim(-max_radius, max_radius)
        ax.set_ylim(-max_radius, max_radius)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        
        # Add legend for windings
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor=self.colors['winding_A'],
                  markersize=8, label='Phase A (+)', markeredgewidth=2, markeredgecolor=self.colors['winding_A']),
            Line2D([0], [0], marker='o', color='w', markerfacecolor=self.colors['winding_B'],
                  markersize=8, label='Phase B (+)', markeredgewidth=2, markeredgecolor=self.colors['winding_B']),
            Line2D([0], [0], marker='o', color='w', markerfacecolor=self.colors['winding_C'],
                  markersize=8, label='Phase C (+)', markeredgewidth=2, markeredgecolor=self.colors['winding_C']),
        ]
        ax.legend(handles=legend_elements, loc='upper right', fontsize=8)
    
    def plot_flux_density(self, ax, solution: Dict, rotor_angle: float = 0.0):
        """Plot flux density distribution with contours"""
        ax.clear()
        ax.set_aspect('equal')
        ax.set_title('Flux Density Distribution', fontsize=10, fontweight='bold')
        
        # Create mesh grid
        n_points = 100
        max_r = self.params.stator_outer_diameter / 2
        x = np.linspace(-max_r, max_r, n_points)
        y = np.linspace(-max_r, max_r, n_points)
        X, Y = np.meshgrid(x, y)
        
        # Calculate flux density at each point (simplified interpolation)
        B = np.zeros_like(X)
        fluxes = solution['fluxes']
        
        for i in range(n_points):
            for j in range(n_points):
                r = np.sqrt(X[i, j]**2 + Y[i, j]**2)
                theta = np.arctan2(Y[i, j], X[i, j])
                
                # Determine region and interpolate flux density
                if r < self.params.rotor_inner_diameter / 2:
                    B[i, j] = 0.0  # Shaft
                elif r < self.params.rotor_outer_diameter / 2:
                    # Rotor core
                    B[i, j] = 0.5
                elif r < self.params.rotor_outer_diameter / 2 + self.params.magnet_thickness:
                    # Magnet region
                    pole_idx = int((theta % (2 * np.pi)) / (2 * np.pi) * self.params.num_poles)
                    if pole_idx < len(fluxes['magnets']):
                        airgap_area = self.params.stack_length * self.params.pole_pitch * 1e-6
                        B[i, j] = abs(fluxes['magnets'][pole_idx]) / airgap_area
                elif r < self.params.stator_inner_diameter / 2 + self.params.airgap_length:
                    # Airgap
                    ag_idx = int((theta % (2 * np.pi)) / (2 * np.pi) * len(fluxes['airgap']))
                    if ag_idx < len(fluxes['airgap']):
                        airgap_area = self.params.stack_length * self.params.pole_pitch * 1e-6
                        B[i, j] = abs(fluxes['airgap'][ag_idx]) / airgap_area
                elif r < self.params.stator_inner_diameter / 2 + self.params.slot_depth:
                    # Teeth region
                    slot_idx = int((theta % (2 * np.pi)) / (2 * np.pi) * self.params.num_slots)
                    if slot_idx < len(fluxes['teeth']):
                        tooth_area = self.params.tooth_width * self.params.stack_length * 1e-6
                        B[i, j] = abs(fluxes['teeth'][slot_idx]) / tooth_area
                elif r < self.params.stator_outer_diameter / 2:
                    # Yoke - use actual yoke flux from solution
                    yoke_idx = int((theta % (2 * np.pi)) / (2 * np.pi) * len(fluxes['yoke']))
                    if yoke_idx < len(fluxes['yoke']):
                        yoke_thickness = (self.params.stator_outer_diameter - self.params.stator_inner_diameter) / 2 - self.params.slot_depth
                        yoke_area = yoke_thickness * self.params.stack_length * 1e-6
                        B[i, j] = abs(fluxes['yoke'][yoke_idx]) / yoke_area
                    else:
                        B[i, j] = 0.0
                else:
                    B[i, j] = 0.0
        
        # Plot contours
        levels = np.linspace(0, 1.5, 15)
        contour = ax.contourf(X, Y, B, levels=levels, cmap='RdYlBu_r', alpha=0.8)
        plt.colorbar(contour, ax=ax, label='Flux Density (T)')
        
        # Overlay motor outline
        stator_circle = Circle((0, 0), self.params.stator_inner_diameter / 2,
                              fill=False, edgecolor='black', linewidth=2, linestyle='--')
        rotor_circle = Circle((0, 0), self.params.rotor_outer_diameter / 2,
                             fill=False, edgecolor='black', linewidth=2, linestyle='--')
        ax.add_patch(stator_circle)
        ax.add_patch(rotor_circle)
        
        ax.set_xlim(-max_r, max_r)
        ax.set_ylim(-max_r, max_r)
        ax.set_xlabel('X (mm)', fontsize=8)
        ax.set_ylabel('Y (mm)', fontsize=8)
        ax.tick_params(labelsize=8)
    
    def plot_airgap_flux(self, ax, solution: Dict):
        """Plot airgap flux density vs angular position"""
        ax.clear()
        ax.set_title('Airgap Flux Density', fontsize=10, fontweight='bold')
        
        fluxes = solution['fluxes']
        n_points = len(fluxes['airgap'])
        theta_deg = np.linspace(0, 360, n_points)
        
        # Convert flux to flux density
        airgap_area = self.params.stack_length * self.params.pole_pitch * 1e-6
        B_airgap = [f / airgap_area for f in fluxes['airgap']]
        
        ax.plot(theta_deg, B_airgap, 'b-', linewidth=2, label='Airgap Flux Density')
        ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('Mechanical Angle (degrees)', fontsize=8)
        ax.set_ylabel('Flux Density (T)', fontsize=8)
        ax.legend(fontsize=8)
        ax.tick_params(labelsize=8)
        
        # Mark pole positions
        num_poles = int(self.params.num_poles)
        pole_angles = [360 * i / num_poles for i in range(num_poles)]
        for angle in pole_angles:
            ax.axvline(x=angle, color='r', linestyle=':', alpha=0.3, linewidth=1)
    
    def plot_fea_airgap_flux(self, ax, fea_solver, fea_solution: Dict):
        """
        Plot FEA airgap flux density vs angular position
        
        Args:
            ax: Matplotlib axis
            fea_solver: FEASolver instance
            fea_solution: FEA solution dictionary from FEASolver
        """
        ax.clear()
        ax.set_title('FEA Airgap Flux Density', fontsize=10, fontweight='bold')
        
        # Get airgap flux density distribution from FEA
        n_points = 360
        B_radial = fea_solver.get_airgap_flux(fea_solution, n_points=n_points)
        
        # Check if we got valid data
        if B_radial is None or len(B_radial) == 0:
            ax.text(0.5, 0.5, 'No airgap flux data available',
                   ha='center', va='center', fontsize=12, color='gray')
            ax.axis('off')
            return
        
        theta_deg = np.linspace(0, 360, len(B_radial), endpoint=False)
        
        # Plot flux density
        ax.plot(theta_deg, B_radial, 'b-', linewidth=2, label='Radial Flux Density')
        ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('Mechanical Angle (degrees)', fontsize=8)
        ax.set_ylabel('Radial Flux Density (T)', fontsize=8)
        ax.legend(fontsize=8)
        ax.tick_params(labelsize=8)
        
        # Mark pole positions
        num_poles = int(self.params.num_poles)
        pole_angles = [360 * i / num_poles for i in range(num_poles)]
        for angle in pole_angles:
            ax.axvline(x=angle, color='r', linestyle=':', alpha=0.3, linewidth=1)
        
        # Count zero crossings
        zero_crossings = np.sum(np.diff(np.sign(B_radial)) != 0)
        
        # Add statistics
        B_max = np.max(np.abs(B_radial))
        B_mean = np.mean(np.abs(B_radial))
        stats_text = f'Max: {B_max:.3f} T\nMean: {B_mean:.3f} T\nZero crossings: {zero_crossings}'
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
               fontsize=7, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    def plot_mec_diagram(self, ax, solution: Dict):
        """Plot detailed MEC diagram showing two complete poles with all individual elements"""
        ax.clear()
        ax.set_title('MEC: Two-Pole Detailed Circuit', fontsize=10, fontweight='bold')
        ax.axis('off')
        
        # Get parameters
        num_slots = int(self.params.num_slots)
        num_poles = int(self.params.num_poles)
        fluxes = solution['fluxes']
        
        # Calculate teeth per pole
        teeth_per_pole = num_slots // num_poles
        
        # Drawing parameters
        box_w = 0.06
        box_h = 0.04
        pole_spacing = 0.45
        
        y_pos = {
            'yoke': 0.92,
            'tooth': 0.75,
            'airgap': 0.58,
            'mag_top': 0.42,
            'mag_bot': 0.30,
            'rotor': 0.14
        }
        
        # Get winding configuration
        winding_config = self.geometry.get_winding_positions()
        slot_to_phase = {}
        for phase, windings in winding_config.items():
            for winding in windings:
                slot_to_phase[winding['slot_index']] = {
                    'phase': phase,
                    'polarity': winding['polarity']
                }
        
        phase_colors = {'A': '#DD6B20', 'B': '#38A169', 'C': '#805AD5'}
        
        # Draw two poles
        for pole_idx in range(2):
            pole_x_center = 0.25 + pole_idx * pole_spacing
            pole_label = 'N' if pole_idx == 0 else 'S'
            
            # --- STATOR YOKE (top, horizontal span for this pole) ---
            yoke_left = pole_x_center - 0.15
            yoke_right = pole_x_center + 0.15
            ax.plot([yoke_left, yoke_right], [y_pos['yoke'], y_pos['yoke']], 
                   'k-', linewidth=3)
            rect = plt.Rectangle((pole_x_center - 0.08, y_pos['yoke'] - box_h/2), 
                                0.16, box_h, facecolor='lightcoral', edgecolor='black', linewidth=2)
            ax.add_patch(rect)
            ax.text(pole_x_center, y_pos['yoke'], 'R_yoke', ha='center', va='center', 
                   fontsize=6, fontweight='bold')
            
            # --- INDIVIDUAL TEETH for this pole ---
            # Show 3 teeth per pole to demonstrate individual elements
            teeth_to_show = min(3, teeth_per_pole)
            tooth_spacing = 0.08
            tooth_start_x = pole_x_center - (teeth_to_show - 1) * tooth_spacing / 2
            
            for t in range(teeth_to_show):
                tooth_x = tooth_start_x + t * tooth_spacing
                slot_idx = pole_idx * teeth_per_pole + int(t * teeth_per_pole / teeth_to_show)
                
                # Get phase info
                phase_info = slot_to_phase.get(slot_idx, {'phase': 'A', 'polarity': 1})
                phase = phase_info['phase']
                polarity = phase_info['polarity']
                
                # Tooth reluctance
                rect = plt.Rectangle((tooth_x - box_w/2, y_pos['tooth'] - box_h/2), 
                                    box_w, box_h, 
                                    facecolor=phase_colors.get(phase, 'yellow'), 
                                    edgecolor='black', linewidth=1.5, alpha=0.7)
                ax.add_patch(rect)
                ax.text(tooth_x, y_pos['tooth'], f'T{slot_idx}', 
                       ha='center', va='center', fontsize=5, fontweight='bold')
                
                # Phase and polarity label
                pol_sign = '+' if polarity > 0 else '-'
                ax.text(tooth_x, y_pos['tooth'] - box_h/2 - 0.015, 
                       f'{phase}{pol_sign}', ha='center', va='top', fontsize=4,
                       color=phase_colors.get(phase, 'black'))
                
                # Connect yoke to tooth
                ax.plot([tooth_x, tooth_x], [y_pos['yoke'] - box_h/2, y_pos['tooth'] + box_h/2], 
                       'k-', linewidth=1.2)
                
                # Airgap reluctance
                rect = plt.Rectangle((tooth_x - box_w/2, y_pos['airgap'] - box_h/2), 
                                    box_w, box_h, 
                                    facecolor='lightblue', edgecolor='black', linewidth=1)
                ax.add_patch(rect)
                ax.text(tooth_x, y_pos['airgap'], 'R_ag', ha='center', va='center', fontsize=4)
                
                # Connect tooth to airgap
                ax.plot([tooth_x, tooth_x], [y_pos['tooth'] - box_h/2, y_pos['airgap'] + box_h/2], 
                       'k-', linewidth=1.2)
                
                # Connect airgap to magnet top
                ax.plot([tooth_x, tooth_x], [y_pos['airgap'] - box_h/2, y_pos['mag_top'] + box_h/2], 
                       'k-', linewidth=1.2)
            
            # --- MAGNET ELEMENTS (2 per pole: top and bottom halves) ---
            mag_color = 'pink' if pole_idx == 0 else 'lightcyan'
            mmf_sign = '+' if pole_idx == 0 else '-'
            
            # Magnet top half (MMF source in parallel with reluctance)
            mag_x_left = pole_x_center - 0.04
            mag_x_right = pole_x_center + 0.04
            mag_y = (y_pos['mag_top'] + y_pos['mag_bot']) / 2
            
            # MMF source (circle)
            ax.add_patch(plt.Circle((mag_x_left, mag_y), 0.025, 
                                   facecolor=mag_color, edgecolor='black', linewidth=2))
            ax.text(mag_x_left, mag_y, pole_label, ha='center', va='center', 
                   fontsize=6, fontweight='bold', color='white')
            ax.text(mag_x_left - 0.06, mag_y, f'{mmf_sign}MMF', ha='right', va='center', fontsize=5)
            
            # Magnet reluctance (box)
            rect = plt.Rectangle((mag_x_right - box_w/2, mag_y - box_h/2), 
                                box_w, box_h, facecolor=mag_color, edgecolor='black', linewidth=1.5)
            ax.add_patch(rect)
            ax.text(mag_x_right, mag_y, 'R_m', ha='center', va='center', fontsize=5, fontweight='bold')
            
            # Horizontal connections at top and bottom of magnet
            ax.plot([mag_x_left - 0.025, mag_x_right + box_w/2], 
                   [y_pos['mag_top'], y_pos['mag_top']], 'k-', linewidth=1.5)
            ax.plot([mag_x_left - 0.025, mag_x_right + box_w/2], 
                   [y_pos['mag_bot'], y_pos['mag_bot']], 'k-', linewidth=1.5)
            
            # Vertical lines for MMF and R_m
            ax.plot([mag_x_left, mag_x_left], [y_pos['mag_top'], mag_y - 0.025], 'k-', linewidth=1.5)
            ax.plot([mag_x_left, mag_x_left], [mag_y + 0.025, y_pos['mag_bot']], 'k-', linewidth=1.5)
            ax.plot([mag_x_right, mag_x_right], [y_pos['mag_top'], mag_y - box_h/2], 'k-', linewidth=1.5)
            ax.plot([mag_x_right, mag_x_right], [mag_y + box_h/2, y_pos['mag_bot']], 'k-', linewidth=1.5)
            
            # --- ROTOR BACKIRON (bottom) ---
            rotor_left = pole_x_center - 0.12
            rotor_right = pole_x_center + 0.12
            ax.plot([rotor_left, rotor_right], [y_pos['rotor'], y_pos['rotor']], 
                   'darkgreen', linewidth=3)
            rect = plt.Rectangle((pole_x_center - 0.08, y_pos['rotor'] - box_h/2), 
                                0.16, box_h, facecolor='lightgreen', edgecolor='black', linewidth=2)
            ax.add_patch(rect)
            ax.text(pole_x_center, y_pos['rotor'], 'R_rotor', ha='center', va='center', 
                   fontsize=6, fontweight='bold')
            
            # Connect magnet bottom to rotor
            ax.plot([pole_x_center, pole_x_center], [y_pos['mag_bot'], y_pos['rotor'] + box_h/2], 
                   'darkgreen', linewidth=2)
        
        # Connect the two poles' yokes together at the top
        ax.plot([0.25 + 0.15, 0.25 + pole_spacing - 0.15], [y_pos['yoke'], y_pos['yoke']], 
               'purple', linewidth=2.5, linestyle='--')
        ax.annotate('', xy=(0.25 + pole_spacing - 0.15, y_pos['yoke']), 
                   xytext=(0.25 + 0.15, y_pos['yoke']),
                   arrowprops=dict(arrowstyle='->', lw=2, color='purple'))
        
        # Connect the two poles' rotors together at the bottom
        ax.plot([0.25 - 0.12, 0.25 + pole_spacing + 0.12], [y_pos['rotor'], y_pos['rotor']], 
               'darkgreen', linewidth=2.5, linestyle='--')
        ax.annotate('', xy=(0.25 - 0.12, y_pos['rotor']), 
                   xytext=(0.25 + pole_spacing + 0.12, y_pos['rotor']),
                   arrowprops=dict(arrowstyle='->', lw=2, color='darkgreen'))
        
        # Add legend
        ax.text(0.02, 0.50, 'Components:', ha='left', va='top', fontsize=6, fontweight='bold')
        ax.text(0.02, 0.46, '• N/S: Magnet MMF', ha='left', va='top', fontsize=5)
        ax.text(0.02, 0.42, '• R_m: Magnet reluctance', ha='left', va='top', fontsize=5)
        ax.text(0.02, 0.38, '• R_ag: Airgap reluctance', ha='left', va='top', fontsize=5)
        ax.text(0.02, 0.34, '• T#: Tooth reluctance', ha='left', va='top', fontsize=5)
        ax.text(0.02, 0.30, '• R_yoke: Stator yoke', ha='left', va='top', fontsize=5)
        ax.text(0.02, 0.26, '• R_rotor: Rotor backiron', ha='left', va='top', fontsize=5)
        
        ax.text(0.02, 0.20, 'Flux Path:', ha='left', va='top', fontsize=6, fontweight='bold')
        ax.text(0.02, 0.16, 'Magnet → Airgap → Tooth', ha='left', va='top', fontsize=5)
        ax.text(0.02, 0.12, '→ Yoke → Other Pole', ha='left', va='top', fontsize=5)
        ax.text(0.02, 0.08, '→ Rotor (return path)', ha='left', va='top', fontsize=5)
        
        # Phase legend
        ax.text(0.75, 0.06, 'Phases:', ha='left', va='top', fontsize=6, fontweight='bold')
        for i, (phase, color) in enumerate(phase_colors.items()):
            ax.add_patch(plt.Rectangle((0.75, 0.02 - i*0.03), 0.015, 0.015, 
                                       facecolor=color, edgecolor='black', alpha=0.7))
            ax.text(0.77, 0.0275 - i*0.03, f'{phase}', ha='left', va='center', fontsize=5)
        
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
    
    def plot_fea_results(self, ax, fea_solution: Dict):
        """
        Plot FEA flux density results
        
        Args:
            ax: Matplotlib axis
            fea_solution: FEA solution dictionary from FEASolver
        """
        ax.clear()
        ax.set_title('FEA Flux Density', fontsize=10, fontweight='bold')
        ax.set_xlabel('X (m)', fontsize=8)
        ax.set_ylabel('Y (m)', fontsize=8)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        
        mesh = fea_solution['mesh']
        nodes = mesh['nodes']
        elements = mesh['elements']
        B_nodes = fea_solution['flux_density_nodes']  # Use nodal values
        
        # Calculate flux density magnitude at nodes
        B_mag = np.linalg.norm(B_nodes, axis=1)
        
        # Create triangular mesh plot
        from matplotlib.tri import Triangulation
        
        # Create triangulation
        tri = Triangulation(nodes[:, 0], nodes[:, 1], elements)
        
        # Use percentile-based levels for better contrast
        # This makes low and high flux regions more visible
        vmin = np.percentile(B_mag, 5)  # Ignore very low noise
        vmax = np.percentile(B_mag, 99)  # Ignore outliers
        
        # Check if we have enough variation to create meaningful contours
        if vmax - vmin < 1e-6:
            # Flux is essentially uniform - show as flat color
            ax.tripcolor(tri, B_mag, cmap='RdYlBu_r', alpha=0.8, shading='gouraud')
            cbar = plt.colorbar(ax.collections[-1], ax=ax, label='|B| (T)')
            cbar.ax.tick_params(labelsize=7)
        else:
            # Create non-linear levels for better contrast in different regions
            levels = np.concatenate([
                np.linspace(vmin, vmax*0.3, 8),    # More resolution in low field
                np.linspace(vmax*0.3, vmax*0.7, 6), # Medium field
                np.linspace(vmax*0.7, vmax, 6)      # High field (magnets, teeth)
            ])
            
            # Ensure levels are strictly increasing
            levels = np.unique(levels)
            if len(levels) < 2:
                levels = np.linspace(vmin, vmax, 10)
            
            # Plot flux density contours
            contourf = ax.tricontourf(tri, B_mag, levels=levels, cmap='RdYlBu_r', alpha=0.8)
            
            # Add colorbar
            cbar = plt.colorbar(contourf, ax=ax, label='|B| (T)')
            cbar.ax.tick_params(labelsize=7)
        
        # Add magnetic field lines (vector potential contours) if available
        if 'vector_potential' in fea_solution:
            A_elements = fea_solution['vector_potential']
            
            # Interpolate A to nodes for smoother contours
            A_nodes = np.zeros(len(nodes))
            node_count = np.zeros(len(nodes))
            
            for elem_idx, elem in enumerate(elements):
                for node_idx in elem.astype(int):
                    A_nodes[node_idx] += A_elements[elem_idx]
                    node_count[node_idx] += 1
            
            for i in range(len(nodes)):
                if node_count[i] > 0:
                    A_nodes[i] /= node_count[i]
            
            # Plot field lines as contours of A
            # Magnetic field lines follow contours of constant A (flux lines)
            A_min = np.min(A_nodes)
            A_max = np.max(A_nodes)
            A_range = A_max - A_min
            
            if A_range > 1e-10:  # Only plot if we have non-zero field
                # Use logarithmic-like spacing to capture both high and low field regions
                # This ensures field lines are visible in airgap, teeth, and yoke
                num_lines = 30
                
                # Create levels with more density near extremes (magnets) and middle (airgap)
                levels_pos = np.concatenate([
                    np.linspace(0, A_max * 0.3, 8),
                    np.linspace(A_max * 0.3, A_max * 0.7, 6),
                    np.linspace(A_max * 0.7, A_max, 7)
                ])
                levels_neg = -levels_pos[::-1]
                A_levels = np.concatenate([levels_neg[:-1], levels_pos])
                
                # Remove duplicates and sort
                A_levels = np.unique(A_levels)
                
                # Plot field lines as white contours (more visible on color background)
                contour_lines = ax.tricontour(tri, A_nodes, levels=A_levels, 
                                              colors='white', linewidths=0.8, alpha=0.6)
                
                # Add thicker lines for key flux paths (every 5th line)
                key_levels = A_levels[::5]
                if len(key_levels) > 0:
                    ax.tricontour(tri, A_nodes, levels=key_levels, 
                                 colors='black', linewidths=1.2, alpha=0.7)
        
        # Overlay motor outline (convert mm to meters)
        mm_to_m = 0.001
        r_stator = self.params.stator_outer_diameter / 2 * mm_to_m
        r_rotor = self.params.rotor_inner_diameter / 2 * mm_to_m
        
        stator_circle = Circle((0, 0), r_stator, fill=False, 
                              edgecolor='black', linewidth=2, linestyle='--')
        rotor_circle = Circle((0, 0), r_rotor, fill=False,
                             edgecolor='black', linewidth=1.5, linestyle='--')
        ax.add_patch(stator_circle)
        ax.add_patch(rotor_circle)
        
        # Set limits
        max_r = r_stator * 1.1
        ax.set_xlim(-max_r, max_r)
        ax.set_ylim(-max_r, max_r)
    
    def plot_fea_mesh(self, ax, mesh_data: Dict, rotor_angle: float = 0.0):
        """
        Plot FEA mesh with geometry overlay
        
        Args:
            ax: Matplotlib axis
            mesh_data: Mesh dictionary from FEASolver
            rotor_angle: Rotor position in degrees for geometry overlay
        """
        ax.clear()
        ax.set_title('FEA Mesh with Geometry', fontsize=10, fontweight='bold')
        ax.set_xlabel('X (m)', fontsize=8)
        ax.set_ylabel('Y (m)', fontsize=8)
        ax.set_aspect('equal')
        
        nodes = mesh_data['nodes']
        elements = mesh_data['elements']
        
        # Plot mesh edges first (in background)
        from matplotlib.tri import Triangulation
        tri = Triangulation(nodes[:, 0], nodes[:, 1], elements)
        ax.triplot(tri, 'k-', linewidth=0.3, alpha=0.3)
        
        # Plot nodes
        ax.plot(nodes[:, 0], nodes[:, 1], 'b.', markersize=0.5, alpha=0.2)
        
        # Get geometry for overlay (convert mm to meters)
        mm_to_m = 0.001
        stator_geo = self.geometry.generate_stator_geometry()
        rotor_geo = self.geometry.generate_rotor_geometry()
        
        # Overlay geometry on top of mesh
        # Stator outer
        stator_outer = Circle((0, 0), self.params.stator_outer_diameter / 2 * mm_to_m,
                             fill=False, edgecolor='darkblue', linewidth=2, linestyle='--')
        ax.add_patch(stator_outer)
        
        # Stator bore
        stator_bore = Circle((0, 0), self.params.stator_inner_diameter / 2 * mm_to_m,
                            fill=False, edgecolor='darkblue', linewidth=2, linestyle='--')
        ax.add_patch(stator_bore)
        
        # Plot slots outline (convert vertices to meters)
        for slot in stator_geo['slots']:
            vertices = [[x * mm_to_m, y * mm_to_m] for x, y in slot['vertices']]
            slot_poly = Polygon(vertices, fill=False, edgecolor='green',
                              linewidth=1.5, linestyle='--', alpha=0.7)
            ax.add_patch(slot_poly)
        
        # Rotor outer
        rotor_outer = Circle((0, 0), self.params.rotor_outer_diameter / 2 * mm_to_m,
                            fill=False, edgecolor='red', linewidth=2, linestyle='--')
        ax.add_patch(rotor_outer)
        
        # Plot magnets outline (convert vertices to meters and rotate)
        angle_rad = np.radians(rotor_angle)
        for magnet in rotor_geo['magnets']:
            vertices = magnet['vertices']
            # Convert to meters and rotate vertices
            rotated_vertices = []
            for x, y in vertices:
                x_m = x * mm_to_m
                y_m = y * mm_to_m
                x_rot = x_m * np.cos(angle_rad) - y_m * np.sin(angle_rad)
                y_rot = x_m * np.sin(angle_rad) + y_m * np.cos(angle_rad)
                rotated_vertices.append([x_rot, y_rot])
            
            magnet_poly = Polygon(rotated_vertices, fill=False,
                                edgecolor='darkred', linewidth=1.5, linestyle='--', alpha=0.7)
            ax.add_patch(magnet_poly)
        
        # Shaft
        shaft = Circle((0, 0), self.params.rotor_inner_diameter / 2 * mm_to_m,
                      fill=False, edgecolor='gray', linewidth=2, linestyle='--')
        ax.add_patch(shaft)
        
        # Add info text
        info_text = f"Nodes: {mesh_data['num_nodes']}\nElements: {mesh_data['num_elements']}"
        ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
               fontsize=8, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Add legend
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='k', linewidth=0.3, alpha=0.3, label='FEA Mesh'),
            Line2D([0], [0], color='darkblue', linewidth=2, linestyle='--', label='Stator'),
            Line2D([0], [0], color='green', linewidth=1.5, linestyle='--', label='Slots'),
            Line2D([0], [0], color='red', linewidth=2, linestyle='--', label='Rotor'),
            Line2D([0], [0], color='darkred', linewidth=1.5, linestyle='--', label='Magnets')
        ]
        ax.legend(handles=legend_elements, loc='upper right', fontsize=7)
        
        # Set limits (convert to meters for consistency with mesh coordinates)
        mm_to_m = 0.001
        r_stator = self.params.stator_outer_diameter / 2 * mm_to_m
        max_r = r_stator * 1.1
        ax.set_xlim(-max_r, max_r)
        ax.set_ylim(-max_r, max_r)

