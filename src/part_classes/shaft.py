"""
Shaft Component
Rotating shaft for mechanical power transmission
"""

from typing import Optional


class Shaft:
    """
    Rotating shaft
    Mechanical power transmission
    """
    
    def __init__(self):
        # Parent reference for hierarchical access
        self._parent: Optional['Rotor'] = None
        
        # Shaft properties
        self.material: Optional[str] = None  # e.g., "Steel 4140"
        self.diameter: Optional[float] = None  # mm (use shaft_outer_diameter property instead)
        self.length: Optional[float] = None  # mm
        self.yield_strength: Optional[float] = None  # MPa
        
        # Features
        self.keyway: bool = False
        self.spline: bool = False
    
    @property
    def shaft_outer_diameter(self) -> Optional[float]:
        """Calculated: rotor_core_inner_diameter"""
        if not self._parent or not self._parent.core:
            return None
        return self._parent.core.rotor_core_inner_diameter
        
    def __repr__(self):
        return f"Shaft(material={self.material}, diameter={self.shaft_outer_diameter}mm)"
    
    def compute_mass(self) -> float:
        """Calculate shaft mass (kg)"""
        # TODO: Implement based on shaft volume and material density
        return 0.0
    
    def compute_cost(self) -> float:
        """Calculate shaft cost ($)"""
        # TODO: Implement based on mass, material cost, and machining
        return 0.0
