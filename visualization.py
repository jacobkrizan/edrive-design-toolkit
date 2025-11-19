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
        
        # Create subplot layout
        gs = fig.add_gridspec(2, 2, hspace=0.3, wspace=0.3)
        ax1 = fig.add_subplot(gs[0, :])  # Motor cross-section
        ax2 = fig.add_subplot(gs[1, 0])  # Flux density distribution
        ax3 = fig.add_subplot(gs[1, 1])  # Airgap flux density
        
        # Plot motor cross-section
        self.plot_motor_cross_section(ax1, rotor_angle)
        
        # Solve electromagnetic problem
        solution = self.solver.solve_mec(rotor_angle)
        
        # Plot flux density contours
        self.plot_flux_density(ax2, solution, rotor_angle)
        
        # Plot airgap flux density
        self.plot_airgap_flux(ax3, solution)
        
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
        
        # Plot slots with windings
        for i, slot in enumerate(stator_geo['slots']):
            # Slot body
            slot_poly = Polygon(slot['vertices'], fill=True, 
                              color='white', edgecolor='black', linewidth=0.5)
            ax.add_patch(slot_poly)
            
            # Add winding indication
            center = slot['center']
            for phase, windings in winding_config.items():
                for winding in windings:
                    if winding['slot_index'] == i:
                        color = self.colors[f'winding_{phase}']
                        marker = 'o' if winding['polarity'] > 0 else 'x'
                        ax.plot(center[0], center[1], marker=marker, 
                               color=color, markersize=6, markeredgewidth=2)
                        break
        
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
                    # Yoke
                    B[i, j] = 1.0
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
