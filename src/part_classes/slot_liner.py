"""
Slot Liner Component
Electrical insulation in stator slots
"""

from typing import Optional


class SlotLiner:
    """
    Electrical insulation in stator slots
    Separates windings from core
    """
    
    def __init__(self):
        # Parent reference for hierarchical access
        self._parent: Optional['Stator'] = None
        
        # Liner parameters from Excel
        self.slot_liner_thickness: Optional[float] = None  # mm
        
        # Additional liner properties
        self.material: Optional[str] = None  # e.g., "Nomex", "Mylar"
        self.dielectric_strength: Optional[float] = None  # kV/mm
        self.thermal_conductivity: Optional[float] = None  # W/m-K
    
    @property
    def stator_slots(self) -> Optional[int]:
        """Access parent's stator_slots for hierarchical calculations"""
        return self._parent.stator_slots if self._parent else None
        
    def __repr__(self):
        return f"SlotLiner(material={self.material}, thickness={self.slot_liner_thickness}mm)"
    
    def compute_mass(self) -> float:
        """Calculate slot liner mass (kg)"""
        # TODO: Implement based on liner area and material density
        return 0.0
    
    def compute_cost(self) -> float:
        """Calculate slot liner cost ($)"""
        # TODO: Implement based on mass and material cost
        return 0.0
