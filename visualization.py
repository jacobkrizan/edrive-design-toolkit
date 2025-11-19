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
from motor_parameters import MotorParameters, PerformanceMetrics
from geometry import MotorGeometry
from magnetic_circuit import MagneticCircuitSolver


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
    
    def plot_mec_diagram(self, ax, solution: Dict):
        """Plot magnetic equivalent circuit diagram showing two poles with yoke and rotor backiron connections"""
        ax.clear()
        ax.set_title('Magnetic Equivalent Circuit (Two Poles)', fontsize=10, fontweight='bold')
        ax.axis('off')
        
        # Get reluctances
        reluctances = solution['reluctances']
        
        # Drawing parameters for two-pole circuit
        box_width = 0.12
        box_height = 0.06
        
        # Positions for left pole (North) and right pole (South)
        x_left = 0.25
        x_right = 0.75
        
        y_positions = {
            'yoke_top': 0.90,
            'tooth': 0.72,
            'airgap': 0.54,
            'magnet_top': 0.38,
            'magnet_bottom': 0.23,
            'rotor_back': 0.08
        }
        
        # Calculate MMF value
        mmf_value = self.params.magnet_br * self.params.magnet_thickness * 1e-3 / (4e-7 * np.pi * self.params.magnet_mu_r)
        
        # Helper function to format reluctance values
        def format_reluctance(r_value):
            if r_value > 1e6:
                return f'{r_value/1e6:.2f}M'
            elif r_value > 1e3:
                return f'{r_value/1e3:.0f}k'
            else:
                return f'{r_value:.0f}'
        
        # Draw LEFT POLE (North)
        # Tooth reluctance (top)
        rect = plt.Rectangle((x_left - box_width/2, y_positions['tooth'] - box_height/2), 
                            box_width, box_height, facecolor='lightyellow', edgecolor='black', linewidth=1.5)
        ax.add_patch(rect)
        ax.text(x_left, y_positions['tooth'], 'R_t', ha='center', va='center', fontsize=6, fontweight='bold')
        
        # Winding MMF source (on tooth)
        winding_mmf = self.params.rated_current * 10  # Simplified: turns * current
        winding_x = x_left + 0.13
        ax.add_patch(plt.Circle((winding_x, y_positions['tooth']), 0.025, 
                                facecolor='orange', edgecolor='black', linewidth=1.5))
        ax.text(winding_x, y_positions['tooth'], 'W', ha='center', va='center', 
                fontsize=5, fontweight='bold', color='white')
        ax.text(winding_x + 0.10, y_positions['tooth'], f'{winding_mmf:.0f} AT', 
                ha='left', va='center', fontsize=5)
        
        # Airgap reluctance (below tooth)
        rect = plt.Rectangle((x_left - box_width/2, y_positions['airgap'] - box_height/2), 
                            box_width, box_height, facecolor='lightblue', edgecolor='black', linewidth=1.5)
        ax.add_patch(rect)
        ax.text(x_left, y_positions['airgap'], 'R_ag', ha='center', va='center', fontsize=6, fontweight='bold')
        
        # Magnet section - MMF source (left) and reluctance (right) in parallel
        magnet_y = (y_positions['magnet_top'] + y_positions['magnet_bottom']) / 2
        
        # MMF source circle (left side)
        mmf_x = x_left - 0.045
        ax.add_patch(plt.Circle((mmf_x, magnet_y), 0.035, 
                                facecolor='red', edgecolor='black', linewidth=2))
        ax.text(mmf_x, magnet_y, 'N', ha='center', va='center', 
                fontsize=6, fontweight='bold', color='white')
        ax.text(mmf_x - 0.12, magnet_y, f'+{mmf_value:.0f} AT', 
                ha='right', va='center', fontsize=5)
        
        # Magnet reluctance box (right side)
        r_mag_x = x_left + 0.045
        rect = plt.Rectangle((r_mag_x - box_width/4, magnet_y - box_height/2), 
                            box_width/2, box_height, facecolor='pink', edgecolor='black', linewidth=1.5)
        ax.add_patch(rect)
        ax.text(r_mag_x, magnet_y, 'R_m', ha='center', va='center', fontsize=5, fontweight='bold')
        
        # Draw RIGHT POLE (South)
        # Tooth reluctance (top)
        rect = plt.Rectangle((x_right - box_width/2, y_positions['tooth'] - box_height/2), 
                            box_width, box_height, facecolor='lightyellow', edgecolor='black', linewidth=1.5)
        ax.add_patch(rect)
        ax.text(x_right, y_positions['tooth'], 'R_t', ha='center', va='center', fontsize=6, fontweight='bold')
        
        # Winding MMF source (on tooth)
        winding_x_r = x_right + 0.13
        ax.add_patch(plt.Circle((winding_x_r, y_positions['tooth']), 0.025, 
                                facecolor='orange', edgecolor='black', linewidth=1.5))
        ax.text(winding_x_r, y_positions['tooth'], 'W', ha='center', va='center', 
                fontsize=5, fontweight='bold', color='white')
        ax.text(winding_x_r + 0.10, y_positions['tooth'], f'-{winding_mmf:.0f} AT', 
                ha='left', va='center', fontsize=5)
        
        # Airgap reluctance (below tooth)
        rect = plt.Rectangle((x_right - box_width/2, y_positions['airgap'] - box_height/2), 
                            box_width, box_height, facecolor='lightblue', edgecolor='black', linewidth=1.5)
        ax.add_patch(rect)
        ax.text(x_right, y_positions['airgap'], 'R_ag', ha='center', va='center', fontsize=6, fontweight='bold')
        
        # Magnet section - MMF source (left) and reluctance (right) in parallel
        # MMF source circle (left side)
        mmf_x_r = x_right - 0.045
        ax.add_patch(plt.Circle((mmf_x_r, magnet_y), 0.035, 
                                facecolor='blue', edgecolor='black', linewidth=2))
        ax.text(mmf_x_r, magnet_y, 'S', ha='center', va='center', 
                fontsize=6, fontweight='bold', color='white')
        ax.text(mmf_x_r - 0.12, magnet_y, f'-{mmf_value:.0f} AT', 
                ha='right', va='center', fontsize=5)
        
        # Magnet reluctance box (right side)
        r_mag_x_r = x_right + 0.045
        rect = plt.Rectangle((r_mag_x_r - box_width/4, magnet_y - box_height/2), 
                            box_width/2, box_height, facecolor='lightcyan', edgecolor='black', linewidth=1.5)
        ax.add_patch(rect)
        ax.text(r_mag_x_r, magnet_y, 'R_m', ha='center', va='center', fontsize=5, fontweight='bold')
        
        # STATOR YOKE CONNECTION at top (horizontal)
        yoke_y = y_positions['yoke_top']
        yoke_width = 0.18
        rect = plt.Rectangle((0.5 - yoke_width/2, yoke_y - box_height/2), 
                            yoke_width, box_height, facecolor='lightcoral', edgecolor='black', linewidth=2)
        ax.add_patch(rect)
        ax.text(0.5, yoke_y, 'R_yoke', ha='center', va='center', fontsize=7, fontweight='bold')
        ax.text(0.5, yoke_y + box_height/2 + 0.02, 'Stator Backiron', ha='center', va='bottom', 
                fontsize=6, style='italic', color='darkred')
        
        # ROTOR BACKIRON CONNECTION at bottom (horizontal)
        rotor_y = y_positions['rotor_back']
        rotor_width = 0.18
        rect = plt.Rectangle((0.5 - rotor_width/2, rotor_y - box_height/2), 
                            rotor_width, box_height, facecolor='lightgreen', edgecolor='black', linewidth=2)
        ax.add_patch(rect)
        ax.text(0.5, rotor_y, 'R_rotor', ha='center', va='center', fontsize=7, fontweight='bold')
        ax.text(0.5, rotor_y - box_height/2 - 0.02, 'Rotor Backiron', ha='center', va='top', 
                fontsize=6, style='italic', color='darkgreen')
        
        # FLUX PATH LINES
        # LEFT POLE - Top connection (stator yoke to tooth to airgap to magnet top)
        # Stator yoke to tooth
        ax.plot([0.5 - yoke_width/2, x_left], [yoke_y, yoke_y], 'r-', linewidth=2.5)
        ax.plot([x_left, x_left], [yoke_y, y_positions['tooth'] + box_height/2], 'r-', linewidth=2.5)
        ax.annotate('', xy=(x_left, y_positions['tooth'] + box_height/2 - 0.01), 
                   xytext=(x_left, y_positions['tooth'] + box_height/2 + 0.02),
                   arrowprops=dict(arrowstyle='->', lw=2, color='red'))
        
        # Through tooth to airgap
        ax.plot([x_left, x_left], [y_positions['tooth'] - box_height/2, y_positions['airgap'] + box_height/2], 
                'r-', linewidth=2.5)
        ax.annotate('', xy=(x_left, y_positions['airgap'] + box_height/2 - 0.01), 
                   xytext=(x_left, y_positions['airgap'] + box_height/2 + 0.02),
                   arrowprops=dict(arrowstyle='->', lw=2, color='red'))
        
        # Through airgap to magnet top
        ax.plot([x_left, x_left], [y_positions['airgap'] - box_height/2, y_positions['magnet_top']], 
                'r-', linewidth=2.5)
        
        # Horizontal bus at magnet top connecting MMF and R_m in parallel
        ax.plot([mmf_x - 0.035, r_mag_x + box_width/4], [y_positions['magnet_top'], y_positions['magnet_top']], 
                'k-', linewidth=2)
        
        # LEFT POLE - Bottom connection (magnet bottom to rotor backiron)
        # Horizontal bus at magnet bottom
        ax.plot([mmf_x - 0.035, r_mag_x + box_width/4], [y_positions['magnet_bottom'], y_positions['magnet_bottom']], 
                'k-', linewidth=2)
        
        # Vertical down from magnet bottom to rotor
        ax.plot([x_left, x_left], [y_positions['magnet_bottom'], rotor_y + box_height/2], 
                'darkgreen', linewidth=2.5)
        ax.plot([x_left, 0.5 - rotor_width/2], [rotor_y, rotor_y], 'darkgreen', linewidth=2.5)
        
        # RIGHT POLE - Top connection (stator yoke to tooth to airgap to magnet top)
        # Stator yoke to tooth
        ax.plot([0.5 + yoke_width/2, x_right], [yoke_y, yoke_y], 'b-', linewidth=2.5)
        ax.plot([x_right, x_right], [yoke_y, y_positions['tooth'] + box_height/2], 'b-', linewidth=2.5)
        ax.annotate('', xy=(x_right, y_positions['tooth'] + box_height/2 - 0.01), 
                   xytext=(x_right, y_positions['tooth'] + box_height/2 + 0.02),
                   arrowprops=dict(arrowstyle='->', lw=2, color='blue'))
        
        # Through tooth to airgap
        ax.plot([x_right, x_right], [y_positions['tooth'] - box_height/2, y_positions['airgap'] + box_height/2], 
                'b-', linewidth=2.5)
        ax.annotate('', xy=(x_right, y_positions['airgap'] + box_height/2 - 0.01), 
                   xytext=(x_right, y_positions['airgap'] + box_height/2 + 0.02),
                   arrowprops=dict(arrowstyle='->', lw=2, color='blue'))
        
        # Through airgap to magnet top
        ax.plot([x_right, x_right], [y_positions['airgap'] - box_height/2, y_positions['magnet_top']], 
                'b-', linewidth=2.5)
        
        # Horizontal bus at magnet top
        ax.plot([mmf_x_r - 0.035, r_mag_x_r + box_width/4], [y_positions['magnet_top'], y_positions['magnet_top']], 
                'k-', linewidth=2)
        
        # RIGHT POLE - Bottom connection (magnet bottom to rotor backiron)
        # Horizontal bus at magnet bottom
        ax.plot([mmf_x_r - 0.035, r_mag_x_r + box_width/4], [y_positions['magnet_bottom'], y_positions['magnet_bottom']], 
                'k-', linewidth=2)
        
        # Vertical down from magnet bottom to rotor
        ax.plot([x_right, x_right], [y_positions['magnet_bottom'], rotor_y + box_height/2], 
                'darkgreen', linewidth=2.5)
        ax.plot([x_right, 0.5 + rotor_width/2], [rotor_y, rotor_y], 'darkgreen', linewidth=2.5)
        
        # STATOR YOKE FLUX ARROW (showing circulation from left to right at TOP)
        ax.annotate('', xy=(0.5 + yoke_width/2 - 0.02, yoke_y), 
                   xytext=(0.5 - yoke_width/2 + 0.02, yoke_y),
                   arrowprops=dict(arrowstyle='->', lw=2.5, color='purple'))
        
        # ROTOR BACKIRON FLUX ARROW (right to left - return path at BOTTOM)
        ax.annotate('', xy=(0.5 - rotor_width/2 + 0.02, rotor_y), 
                   xytext=(0.5 + rotor_width/2 - 0.02, rotor_y),
                   arrowprops=dict(arrowstyle='->', lw=2.5, color='darkgreen'))
        
        # Add complete loop indicator
        ax.text(0.98, 0.95, 'Complete\nFlux Loop', ha='right', va='top', fontsize=7, fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.6))
        
        # Add legend
        ax.text(0.02, 0.60, 'Circuit Elements:', ha='left', va='top', fontsize=6, fontweight='bold')
        ax.text(0.02, 0.56, '• N/S: Magnet MMF (|| R_m)', ha='left', va='top', fontsize=5, color='red')
        ax.text(0.02, 0.52, '• W: Winding MMF', ha='left', va='top', fontsize=5, color='orange')
        ax.text(0.02, 0.48, '• Purple: Stator yoke', ha='left', va='top', fontsize=5, color='purple')
        ax.text(0.02, 0.44, '• Green: Rotor backiron', ha='left', va='top', fontsize=5, color='darkgreen')
        
        ax.text(0.02, 0.38, 'Order (top to bottom):', ha='left', va='top', fontsize=6, fontweight='bold')
        ax.text(0.02, 0.34, '1. Stator yoke (top)', ha='left', va='top', fontsize=5)
        ax.text(0.02, 0.30, '2. Tooth + Winding MMF', ha='left', va='top', fontsize=5)
        ax.text(0.02, 0.26, '3. Airgap', ha='left', va='top', fontsize=5)
        ax.text(0.02, 0.22, '4. Magnet (MMF || R_m)', ha='left', va='top', fontsize=5)
        ax.text(0.02, 0.18, '5. Rotor backiron (bottom)', ha='left', va='top', fontsize=5)
        
        ax.set_xlim(0, 1)
        ax.set_ylim(0.0, 1.0)
    
    def plot_fea_results(self, ax, fea_solution: Dict):
        """
        Plot FEA flux density results
        
        Args:
            ax: Matplotlib axis
            fea_solution: FEA solution dictionary from FEASolver
        """
        ax.clear()
        ax.set_title('FEA Flux Density', fontsize=10, fontweight='bold')
        ax.set_xlabel('X (mm)', fontsize=8)
        ax.set_ylabel('Y (mm)', fontsize=8)
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
        
        # Plot flux density contours
        levels = np.linspace(0, np.max(B_mag), 20)
        contourf = ax.tricontourf(tri, B_mag, levels=levels, cmap='RdYlBu_r', alpha=0.8)
        
        # Add colorbar
        cbar = plt.colorbar(contourf, ax=ax, label='|B| (T)')
        cbar.ax.tick_params(labelsize=7)
        
        # Overlay motor outline
        r_stator = self.params.stator_outer_diameter / 2
        r_rotor = self.params.rotor_inner_diameter / 2
        
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
        ax.set_xlabel('X (mm)', fontsize=8)
        ax.set_ylabel('Y (mm)', fontsize=8)
        ax.set_aspect('equal')
        
        nodes = mesh_data['nodes']
        elements = mesh_data['elements']
        
        # Plot mesh edges first (in background)
        from matplotlib.tri import Triangulation
        tri = Triangulation(nodes[:, 0], nodes[:, 1], elements)
        ax.triplot(tri, 'k-', linewidth=0.3, alpha=0.3)
        
        # Plot nodes
        ax.plot(nodes[:, 0], nodes[:, 1], 'b.', markersize=0.5, alpha=0.2)
        
        # Get geometry for overlay
        stator_geo = self.geometry.generate_stator_geometry()
        rotor_geo = self.geometry.generate_rotor_geometry()
        
        # Overlay geometry on top of mesh
        # Stator outer
        stator_outer = Circle((0, 0), self.params.stator_outer_diameter / 2,
                             fill=False, edgecolor='darkblue', linewidth=2, linestyle='--')
        ax.add_patch(stator_outer)
        
        # Stator bore
        stator_bore = Circle((0, 0), self.params.stator_inner_diameter / 2,
                            fill=False, edgecolor='darkblue', linewidth=2, linestyle='--')
        ax.add_patch(stator_bore)
        
        # Plot slots outline
        for slot in stator_geo['slots']:
            vertices = slot['vertices']
            slot_poly = Polygon(vertices, fill=False, edgecolor='green',
                              linewidth=1.5, linestyle='--', alpha=0.7)
            ax.add_patch(slot_poly)
        
        # Rotor outer
        rotor_outer = Circle((0, 0), self.params.rotor_outer_diameter / 2,
                            fill=False, edgecolor='red', linewidth=2, linestyle='--')
        ax.add_patch(rotor_outer)
        
        # Plot magnets outline
        angle_rad = np.radians(rotor_angle)
        for magnet in rotor_geo['magnets']:
            vertices = magnet['vertices']
            # Rotate vertices
            rotated_vertices = []
            for x, y in vertices:
                x_rot = x * np.cos(angle_rad) - y * np.sin(angle_rad)
                y_rot = x * np.sin(angle_rad) + y * np.cos(angle_rad)
                rotated_vertices.append([x_rot, y_rot])
            
            magnet_poly = Polygon(rotated_vertices, fill=False,
                                edgecolor='darkred', linewidth=1.5, linestyle='--', alpha=0.7)
            ax.add_patch(magnet_poly)
        
        # Shaft
        shaft = Circle((0, 0), self.params.rotor_inner_diameter / 2,
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
        
        # Set limits
        r_stator = self.params.stator_outer_diameter / 2
        max_r = r_stator * 1.1
        ax.set_xlim(-max_r, max_r)
        ax.set_ylim(-max_r, max_r)

