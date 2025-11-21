"""
Motor Display Module
Visualizes surface mount permanent magnet motor geometry
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge, Rectangle, Polygon
from matplotlib.collections import PatchCollection
from typing import Optional
from part_classes import Motor


class MotorDisplay:
    """
    Visualize surface mount permanent magnet motor cross-section
    Shows stator, rotor, magnets, slots, and airgap
    """
    
    def __init__(self, motor: Motor):
        """
        Initialize motor display
        
        Args:
            motor: Motor object with geometry parameters
        """
        self.motor = motor
        self.fig = None
        self.ax = None
        
    def plot(self, show_dimensions: bool = True, show_centerlines: bool = True):
        """
        Create motor cross-section plot
        
        Args:
            show_dimensions: Whether to display dimension annotations
            show_centerlines: Whether to show centerlines and reference axes
        """
        # Create figure
        self.fig, self.ax = plt.subplots(figsize=(12, 12))
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        
        # Only show labels if dimensions are shown
        if show_dimensions:
            self.ax.set_xlabel('X (mm)', fontsize=12)
            self.ax.set_ylabel('Y (mm)', fontsize=12)
            self.ax.set_title('Surface Mount Permanent Magnet Motor - Cross Section', 
                             fontsize=14, fontweight='bold')
        else:
            self.ax.set_xlabel('')
            self.ax.set_ylabel('')
            self.ax.set_title('')
        
        # Draw components from outside-in
        self._draw_stator_core()
        self._draw_stator_slots()
        self._draw_windings()
        self._draw_airgap()
        self._draw_rotor_core()
        self._draw_magnets()
        self._draw_shaft()
        
        if show_centerlines:
            self._draw_centerlines()
        
        if show_dimensions:
            self._draw_dimensions()
        
        # Set axis limits based on stator outer diameter
        if self.motor.stator.core.stator_core_outer_diameter:
            limit = self.motor.stator.core.stator_core_outer_diameter / 2 * 1.2
            self.ax.set_xlim(-limit, limit)
            self.ax.set_ylim(-limit, limit)
        
        plt.tight_layout()
        return self.fig, self.ax
    
    def _draw_stator_core(self):
        """Draw stator core (outer yoke and teeth)"""
        stator = self.motor.stator
        core = stator.core
        
        if not core.stator_core_outer_diameter or not core.stator_core_inner_diameter:
            return
        
        # Outer diameter circle
        outer_radius = core.stator_core_outer_diameter / 2
        inner_radius = core.stator_core_inner_diameter / 2
        
        # Draw stator yoke (annular region)
        outer_circle = Circle((0, 0), outer_radius, 
                             color='lightgray', ec='navy', linewidth=2, zorder=1)
        self.ax.add_patch(outer_circle)
        
        # Inner circle (will be covered by slots)
        inner_circle = Circle((0, 0), inner_radius, 
                             color='white', ec='navy', linewidth=1.5, zorder=2)
        self.ax.add_patch(inner_circle)
    
    def _draw_stator_slots(self):
        """Draw stator slots and teeth"""
        stator = self.motor.stator
        core = stator.core
        
        if not stator.stator_slots or not core.stator_core_slot_height:
            return
        
        num_slots = int(stator.stator_slots)
        slot_height = core.stator_core_slot_height
        slot_width = core.stator_core_slot_width or 4.0
        slot_opening = core.stator_core_slot_opening or 1.0
        tooth_tip_height = core.stator_core_tooth_tip_height or 0.5
        
        inner_radius = core.stator_core_inner_diameter / 2
        
        # Calculate slot positions
        for i in range(num_slots):
            angle = 2 * np.pi * i / num_slots
            
            # Draw rectangular slot
            self._draw_slot(angle, inner_radius, slot_height, slot_width, 
                          slot_opening, tooth_tip_height)
    
    def _draw_slot(self, angle: float, inner_radius: float, 
                   slot_height: float, slot_width: float,
                   slot_opening: float, tooth_tip_height: float):
        """Draw a single rectangular stator slot"""
        # Slot is a rectangle with constant LINEAR width from bottom to top
        # Bottom of slot is at inner_radius
        # Top of slot is at inner_radius + slot_height
        
        slot_bottom_radius = inner_radius
        slot_top_radius = inner_radius + slot_height
        
        # Calculate DIFFERENT angular half-widths at top and bottom
        # to maintain the same LINEAR width (slot_width)
        # At bottom: arc_length = slot_bottom_radius * angle, so angle = slot_width / slot_bottom_radius
        # At top: arc_length = slot_top_radius * angle, so angle = slot_width / slot_top_radius
        
        half_angle_bottom = slot_width / (2 * slot_bottom_radius)
        half_angle_top = slot_width / (2 * slot_top_radius)
        
        # Four corners of the truly rectangular slot
        # Bottom uses larger angular offset (smaller radius)
        # Top uses smaller angular offset (larger radius)
        # This keeps the arc length (linear width) constant
        
        corners_xy = [
            (slot_bottom_radius * np.cos(angle - half_angle_bottom), 
             slot_bottom_radius * np.sin(angle - half_angle_bottom)),  # Bottom left
            
            (slot_top_radius * np.cos(angle - half_angle_top), 
             slot_top_radius * np.sin(angle - half_angle_top)),      # Top left
            
            (slot_top_radius * np.cos(angle + half_angle_top), 
             slot_top_radius * np.sin(angle + half_angle_top)),      # Top right
            
            (slot_bottom_radius * np.cos(angle + half_angle_bottom), 
             slot_bottom_radius * np.sin(angle + half_angle_bottom))    # Bottom right
        ]
        
        # Create rectangular slot as polygon
        slot_poly = Polygon(corners_xy, facecolor='white', edgecolor='navy', 
                           linewidth=0.5, zorder=3)
        self.ax.add_patch(slot_poly)
        
        # Slot opening (narrow neck) - smaller rectangle at bottom
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
        
        opening_poly = Polygon(opening_corners_xy, facecolor='white', edgecolor='navy',
                              linewidth=0.5, zorder=3)
        self.ax.add_patch(opening_poly)
    
    def _draw_windings(self):
        """Draw individual hairpin winding bars in each slot"""
        stator = self.motor.stator
        core = stator.core
        windings = stator.windings
        
        if not stator.stator_slots or not core.stator_core_slot_height:
            return
        
        if not windings.stator_winding_bars or not windings.stator_winding_wire_height or not windings.stator_winding_wire_width:
            return
        
        num_slots = int(stator.stator_slots)
        slot_height = core.stator_core_slot_height
        slot_width = core.stator_core_slot_width or 4.0
        
        num_bars = int(windings.stator_winding_bars)
        bar_height = windings.stator_winding_wire_height
        bar_width = windings.stator_winding_wire_width
        
        inner_radius = core.stator_core_inner_diameter / 2
        
        # Draw bars in each slot
        for slot_idx in range(num_slots):
            slot_angle = 2 * np.pi * slot_idx / num_slots
            self._draw_hairpin_bars(slot_angle, inner_radius, slot_height, slot_width,
                                   num_bars, bar_height, bar_width)
    
    def _draw_hairpin_bars(self, slot_angle: float, inner_radius: float,
                          slot_height: float, slot_width: float,
                          num_bars: int, bar_height: float, bar_width: float):
        """Draw individual hairpin bars within a slot, perfectly centered"""
        # bar_height is tangential dimension (wire_height from Excel)
        # bar_width is radial dimension (wire_width from Excel)
        # Swap to match physical meaning
        bar_radial = bar_height  # radial size (was wire_height, actually thinner dimension)
        bar_tangential = bar_width  # tangential size (was wire_width, actually wider dimension)
        
        # Calculate radial spacing and starting position
        total_radial_height = num_bars * bar_radial
        radial_spacing = (slot_height - total_radial_height) / (num_bars + 1)
        
        # Starting radius for first bar (bottom of slot + spacing)
        start_radius = inner_radius + radial_spacing + bar_radial / 2
        
        # Calculate slot center line angle (this is where bars should be centered)
        # Bars should be centered along the radial line at slot_angle
        
        # Draw each bar as a small rectangle oriented in slot
        for i in range(num_bars):
            # Center radius of this bar
            bar_center_radius = start_radius + i * (bar_radial + radial_spacing)
            
            # Bar center is along the radial line at slot_angle
            # This ensures perfect centering in the slot
            x_center = bar_center_radius * np.cos(slot_angle)
            y_center = bar_center_radius * np.sin(slot_angle)
            
            # Calculate the four corners of the bar rectangle
            # The bar is oriented with its long axis tangential (perpendicular to radial)
            # and short axis radial (along the slot_angle direction)
            
            # Half dimensions
            half_radial = bar_radial / 2
            half_tangential = bar_tangential / 2
            
            # Create corners in local coordinate system (aligned with radial direction)
            # Then rotate and translate to global position
            
            # Perpendicular direction (tangential)
            perp_angle = slot_angle + np.pi / 2
            
            # Four corners relative to center
            corners = [
                (x_center - half_tangential * np.cos(perp_angle) - half_radial * np.cos(slot_angle),
                 y_center - half_tangential * np.sin(perp_angle) - half_radial * np.sin(slot_angle)),
                
                (x_center + half_tangential * np.cos(perp_angle) - half_radial * np.cos(slot_angle),
                 y_center + half_tangential * np.sin(perp_angle) - half_radial * np.sin(slot_angle)),
                
                (x_center + half_tangential * np.cos(perp_angle) + half_radial * np.cos(slot_angle),
                 y_center + half_tangential * np.sin(perp_angle) + half_radial * np.sin(slot_angle)),
                
                (x_center - half_tangential * np.cos(perp_angle) + half_radial * np.cos(slot_angle),
                 y_center - half_tangential * np.sin(perp_angle) + half_radial * np.sin(slot_angle))
            ]
            
            # Create rectangular bar as polygon
            bar_poly = Polygon(corners, facecolor='orange', edgecolor='darkorange',
                             linewidth=0.5, zorder=4)
            self.ax.add_patch(bar_poly)
    
    def _draw_airgap(self):
        """Draw airgap region"""
        if not self.motor.motor_airgap:
            return
        
        rotor_outer = self.motor.rotor.core.rotor_core_outer_diameter / 2
        stator_inner = self.motor.stator.core.stator_core_inner_diameter / 2
        
        # Airgap as thin annular region
        airgap = Circle((0, 0), stator_inner, 
                       color='white', ec='gray', linewidth=1, 
                       linestyle='--', fill=False, zorder=4)
        self.ax.add_patch(airgap)
    
    def _draw_rotor_core(self):
        """Draw rotor core (back iron)"""
        core = self.motor.rotor.core
        
        if not core.rotor_core_outer_diameter or not core.rotor_core_inner_diameter:
            return
        
        outer_radius = core.rotor_core_outer_diameter / 2
        inner_radius = core.rotor_core_inner_diameter / 2
        
        # Rotor back iron
        rotor_circle = Circle((0, 0), outer_radius, 
                             color='lightgray', ec='black', linewidth=2, zorder=5)
        self.ax.add_patch(rotor_circle)
        
        # Inner bore (for shaft)
        bore = Circle((0, 0), inner_radius, 
                     color='white', ec='black', linewidth=1.5, zorder=6)
        self.ax.add_patch(bore)
    
    def _draw_magnets(self):
        """Draw surface-mounted permanent magnets"""
        magnet = self.motor.rotor.magnet
        core = self.motor.rotor.core
        
        if not magnet.magnet_thickness or not magnet.magnet_arc_percent:
            return
        
        if not self.motor.motor_poles:
            return
        
        num_poles = int(self.motor.motor_poles)
        magnet_thickness = magnet.magnet_thickness
        arc_percent = magnet.magnet_arc_percent / 100.0  # Convert to fraction
        
        # Rotor outer radius
        rotor_radius = core.rotor_core_outer_diameter / 2
        magnet_outer_radius = rotor_radius + magnet_thickness
        
        # Pole pitch angle
        pole_pitch = 360 / num_poles
        magnet_arc_angle = pole_pitch * arc_percent
        
        # Draw each magnet
        for i in range(num_poles):
            pole_center_angle = pole_pitch * i
            
            # Magnet color alternates for N-S poles
            if i % 2 == 0:
                color = 'red'
            else:
                color = 'blue'
            
            # Draw magnet as wedge
            magnet_wedge = Wedge((0, 0), magnet_outer_radius,
                                pole_center_angle - magnet_arc_angle/2,
                                pole_center_angle + magnet_arc_angle/2,
                                width=magnet_thickness,
                                facecolor=color, edgecolor='darkred',
                                linewidth=1.5, alpha=0.7, zorder=7)
            self.ax.add_patch(magnet_wedge)
    
    def _draw_shaft(self):
        """Draw shaft"""
        shaft = self.motor.rotor.shaft
        core = self.motor.rotor.core
        
        if not core.rotor_core_inner_diameter:
            return
        
        shaft_radius = core.rotor_core_inner_diameter / 2
        
        # Shaft
        shaft_circle = Circle((0, 0), shaft_radius,
                             color='dimgray', ec='black', linewidth=2, zorder=8)
        self.ax.add_patch(shaft_circle)
        
        # Center point
        self.ax.plot(0, 0, 'ko', markersize=4, zorder=9)
    
    def _draw_centerlines(self):
        """Draw reference axes and centerlines"""
        stator = self.motor.stator
        if not stator.core.stator_core_outer_diameter:
            return
        
        limit = stator.core.stator_core_outer_diameter / 2 * 1.1
        
        # Horizontal and vertical centerlines
        self.ax.axhline(y=0, color='k', linewidth=0.5, linestyle=':', alpha=0.5)
        self.ax.axvline(x=0, color='k', linewidth=0.5, linestyle=':', alpha=0.5)
        
        # D-axis and Q-axis reference (for first pole)
        if self.motor.motor_poles:
            pole_pitch = 2 * np.pi / self.motor.motor_poles
            
            # D-axis (aligned with magnet center)
            d_axis_x = limit * np.cos(0)
            d_axis_y = limit * np.sin(0)
            self.ax.plot([0, d_axis_x], [0, d_axis_y], 'r--', 
                        linewidth=1, alpha=0.5)
            
            # Q-axis (between magnets)
            q_axis_angle = pole_pitch / 2
            q_axis_x = limit * np.cos(q_axis_angle)
            q_axis_y = limit * np.sin(q_axis_angle)
            self.ax.plot([0, q_axis_x], [0, q_axis_y], 'b--', 
                        linewidth=1, alpha=0.5)
    
    def _draw_dimensions(self):
        """Add dimension annotations"""
        stator = self.motor.stator
        rotor = self.motor.rotor
        
        # Position for dimension text (upper right area)
        if stator.core.stator_core_outer_diameter:
            x_pos = stator.core.stator_core_outer_diameter / 2 * 0.6
            y_start = stator.core.stator_core_outer_diameter / 2 * 0.8
            y_step = stator.core.stator_core_outer_diameter * 0.08
            
            dims = []
            
            # Collect dimensions
            if stator.core.stator_core_outer_diameter:
                dims.append(f"Stator OD: {stator.core.stator_core_outer_diameter:.1f} mm")
            if stator.core.stator_core_inner_diameter:
                dims.append(f"Stator ID: {stator.core.stator_core_inner_diameter:.1f} mm")
            if rotor.core.rotor_core_outer_diameter:
                dims.append(f"Rotor OD: {rotor.core.rotor_core_outer_diameter:.1f} mm")
            if rotor.core.rotor_core_inner_diameter:
                dims.append(f"Rotor ID: {rotor.core.rotor_core_inner_diameter:.1f} mm")
            if self.motor.motor_airgap:
                dims.append(f"Airgap: {self.motor.motor_airgap:.2f} mm")
            if rotor.magnet.magnet_thickness:
                dims.append(f"Magnet Thickness: {rotor.magnet.magnet_thickness:.1f} mm")
            if stator.stator_slots:
                dims.append(f"Slots: {int(stator.stator_slots)}")
            if self.motor.motor_poles:
                dims.append(f"Poles: {int(self.motor.motor_poles)}")
            
            # Draw dimension box
            for i, dim in enumerate(dims):
                self.ax.text(x_pos, y_start - i * y_step, dim,
                           fontsize=9, bbox=dict(boxstyle='round', 
                           facecolor='wheat', alpha=0.8))
    
    def _add_legend(self):
        """Add legend to plot"""
        self.ax.legend(loc='upper left', fontsize=9, framealpha=0.9)
    
    def save(self, filename: str, dpi: int = 300):
        """
        Save plot to file
        
        Args:
            filename: Output filename (e.g., 'motor_cross_section.png')
            dpi: Resolution in dots per inch
        """
        if self.fig:
            self.fig.savefig(filename, dpi=dpi, bbox_inches='tight')
            print(f"Motor plot saved to: {filename}")
    
    def show(self):
        """Display the plot"""
        if self.fig:
            plt.show()


def display_motor(motor: Motor, show: bool = True, save_path: Optional[str] = None):
    """
    Convenience function to display motor geometry
    
    Args:
        motor: Motor object to visualize
        show: Whether to display the plot interactively
        save_path: Optional path to save the plot
        
    Returns:
        MotorDisplay instance
    """
    display = MotorDisplay(motor)
    display.plot(show_dimensions=False, show_centerlines=False)
    
    if save_path:
        display.save(save_path)
    
    if show:
        display.show()
    
    return display
