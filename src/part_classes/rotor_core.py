"""
Rotor Core Component
Laminated steel core of the rotor
"""

from typing import Optional


class RotorCore:
    """
    Laminated steel core of rotor
    Back iron for magnetic flux return path
    """
    
    def __init__(self):
        # Parent reference for hierarchical access
        self._parent: Optional['Rotor'] = None
        
        # Core parameters from Excel
        self.rotor_core_outer_diameter: Optional[float] = None  # mm
        self.rotor_core_inner_diameter: Optional[float] = None  # mm
        
        # Additional core material properties
        self.material: Optional[str] = None  # e.g., "M19 Steel"
        self.lamination_thickness: Optional[float] = None  # mm
        self.stacking_factor: Optional[float] = None  # 0-1
        self.permeability: Optional[float] = None  # relative
        self.back_iron_thickness: Optional[float] = None  # mm
        
    def __repr__(self):
        return f"RotorCore(OD={self.rotor_core_outer_diameter}mm, ID={self.rotor_core_inner_diameter}mm)"
    
    def compute_mass(self) -> float:
        """Calculate rotor core mass (kg)"""
        # TODO: Implement based on geometry and material density
        return 0.0
    
    def compute_cost(self) -> float:
        """Calculate rotor core cost ($)"""
        # TODO: Implement based on mass and material cost
        return 0.0
