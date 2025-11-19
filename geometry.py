"""
Motor Geometry Module
Generates 2D cross-section geometry for rotor, stator, magnets, and windings
"""

import numpy as np
from typing import List, Tuple, Dict
from motor_parameters import MotorParameters


class MotorGeometry:
    """Handles geometric calculations and mesh generation for motor cross-section"""
    
    def __init__(self, params: MotorParameters):
        self.params = params
        self.mu0 = 4 * np.pi * 1e-7  # Permeability of free space
        
    def generate_stator_geometry(self) -> Dict:
        """Generate stator geometry including slots and teeth"""
        geometry = {
            'outer_circle': self._create_circle(self.params.stator_outer_diameter / 2),
            'inner_circle': self._create_circle(self.params.stator_inner_diameter / 2),
            'slots': [],
            'teeth': [],
            'yoke': None
        }
        
        # Generate slots
        num_slots = int(self.params.num_slots)
        for i in range(num_slots):
            angle = 2 * np.pi * i / num_slots
            slot = self._create_slot(angle)
            geometry['slots'].append(slot)
            
        return geometry
    
    def generate_rotor_geometry(self) -> Dict:
        """Generate rotor geometry including shaft and magnet positions"""
        geometry = {
            'outer_circle': self._create_circle(self.params.rotor_outer_diameter / 2),
            'inner_circle': self._create_circle(self.params.rotor_inner_diameter / 2),
            'magnets': [],
            'poles': []
        }
        
        # Generate surface-mounted magnets
        num_poles = int(self.params.num_poles)
        for i in range(num_poles):
            angle = 2 * np.pi * i / num_poles
            magnet = self._create_magnet(angle, i)
            geometry['magnets'].append(magnet)
            
        return geometry
    
    def _create_circle(self, radius: float, n_points: int = 100) -> np.ndarray:
        """Create circle coordinates"""
        theta = np.linspace(0, 2 * np.pi, n_points)
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        return np.column_stack([x, y])
    
    def _create_slot(self, angle: float) -> Dict:
        """Create slot geometry at given angle"""
        r_inner = self.params.stator_inner_diameter / 2
        slot_angle = 2 * np.pi / self.params.num_slots
        
        # Slot opening angle
        opening_angle = self.params.slot_opening / r_inner
        
        # Slot body angle (wider than opening)
        body_width = slot_angle * 0.6  # 60% of slot pitch
        
        # Create slot profile
        angles = [
            angle - opening_angle / 2,
            angle - body_width / 2,
            angle - body_width / 2,
            angle + body_width / 2,
            angle + body_width / 2,
            angle + opening_angle / 2
        ]
        
        radii = [
            r_inner,
            r_inner + 2,
            r_inner + self.params.slot_depth,
            r_inner + self.params.slot_depth,
            r_inner + 2,
            r_inner
        ]
        
        x = [r * np.cos(a) for r, a in zip(radii, angles)]
        y = [r * np.sin(a) for r, a in zip(radii, angles)]
        
        slot = {
            'angle': angle,
            'vertices': np.column_stack([x, y]),
            'center': self._calculate_slot_center(angle, r_inner),
            'area': self._calculate_slot_area(body_width, r_inner)
        }
        
        return slot
    
    def _create_magnet(self, angle: float, pole_index: int) -> Dict:
        """Create surface-mounted magnet geometry"""
        r_rotor = self.params.rotor_outer_diameter / 2
        magnet_thickness = self.params.magnet_thickness
        
        # Magnet arc angle
        pole_angle = 2 * np.pi / self.params.num_poles
        magnet_angle = pole_angle * self.params.magnet_arc_ratio
        
        # Create magnet profile (rectangular in cross-section)
        n_points = 20
        theta = np.linspace(angle - magnet_angle / 2, angle + magnet_angle / 2, n_points)
        
        # Inner arc (on rotor surface)
        x_inner = r_rotor * np.cos(theta)
        y_inner = r_rotor * np.sin(theta)
        
        # Outer arc (magnet outer surface)
        r_outer = r_rotor + magnet_thickness
        x_outer = r_outer * np.cos(theta[::-1])
        y_outer = r_outer * np.sin(theta[::-1])
        
        # Combine into closed polygon
        x = np.concatenate([x_inner, x_outer])
        y = np.concatenate([y_inner, y_outer])
        
        # Determine polarity (alternating N-S)
        polarity = 1 if pole_index % 2 == 0 else -1
        
        magnet = {
            'angle': angle,
            'vertices': np.column_stack([x, y]),
            'center': np.array([r_rotor * np.cos(angle), r_rotor * np.sin(angle)]),
            'polarity': polarity,
            'magnetization': polarity * self.params.magnet_br / self.mu0 / self.params.magnet_mu_r
        }
        
        return magnet
    
    def _calculate_slot_center(self, angle: float, r_inner: float) -> np.ndarray:
        """Calculate center point of slot"""
        r_center = r_inner + self.params.slot_depth / 2
        x = r_center * np.cos(angle)
        y = r_center * np.sin(angle)
        return np.array([x, y])
    
    def _calculate_slot_area(self, width: float, r_inner: float) -> float:
        """Calculate slot cross-sectional area"""
        # Approximate as trapezoid
        r_outer = r_inner + self.params.slot_depth
        w1 = r_inner * width
        w2 = r_outer * width
        area = self.params.slot_depth * (w1 + w2) / 2
        return area
    
    def get_winding_positions(self) -> Dict:
        """Generate winding positions for 3-phase distributed winding"""
        winding_config = {'A': [], 'B': [], 'C': []}
        
        # 3-phase distributed winding
        num_slots = int(self.params.num_slots)
        num_poles = int(self.params.num_poles)
        slots_per_pole = num_slots / num_poles
        slots_per_phase_per_pole = slots_per_pole / 3
        
        for slot_idx in range(num_slots):
            # Determine which phase and polarity
            phase_group = (slot_idx % (num_slots // num_poles)) // (slots_per_pole / 3)
            pole_group = slot_idx // (num_slots // num_poles)
            
            if phase_group < 1:
                phase = 'A'
            elif phase_group < 2:
                phase = 'B'
            else:
                phase = 'C'
            
            # Alternate polarity for coil sides
            polarity = 1 if pole_group % 2 == 0 else -1
            
            angle = 2 * np.pi * slot_idx / num_slots
            winding_config[phase].append({
                'slot_index': slot_idx,
                'angle': angle,
                'polarity': polarity,
                'turns': self.params.turns_per_coil
            })
        
        return winding_config
    
    def get_airgap_mesh(self, n_points: int = 360) -> np.ndarray:
        """Generate mesh points in the airgap for flux calculations"""
        r_airgap = (self.params.stator_inner_diameter + self.params.rotor_outer_diameter) / 4
        theta = np.linspace(0, 2 * np.pi, n_points, endpoint=False)
        x = r_airgap * np.cos(theta)
        y = r_airgap * np.sin(theta)
        return np.column_stack([x, y, theta])
