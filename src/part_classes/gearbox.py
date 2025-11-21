"""
Gearbox Component
Mechanical gearbox for speed reduction
"""

from typing import Optional


class Gearbox:
    """
    Mechanical gearbox
    Provides speed reduction and torque multiplication
    """
    
    def __init__(self):
        # Parent reference for hierarchical access
        self._parent: Optional['DriveUnit'] = None
        
        # Gearbox properties from Excel
        self.gearbox_ratio: Optional[float] = None  # Gear ratio
        
    def __repr__(self):
        return f"Gearbox(ratio={self.gearbox_ratio})"
    
    def compute_mass(self) -> float:
        """Calculate gearbox mass (kg)"""
        # TODO: Implement based on torque rating and gearbox type
        return 0.0
    
    def compute_cost(self) -> float:
        """Calculate gearbox cost ($)"""
        # TODO: Implement based on complexity and torque rating
        return 0.0
