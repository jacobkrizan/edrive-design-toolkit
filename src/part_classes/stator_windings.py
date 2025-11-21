"""
Stator Windings Component
Copper coils in stator slots
"""

from typing import Optional


class StatorWindings:
    """
    Stator winding configuration
    Copper coils in stator slots
    """
    
    def __init__(self):
        # Parent reference for hierarchical access
        self._parent: Optional['Stator'] = None
        
        # Winding parameters from Excel
        self.stator_winding_wire_height: Optional[float] = None  # mm
        self.stator_winding_wire_width: Optional[float] = None  # mm
        self.stator_winding_bars: Optional[int] = None  # number of bars per slot
        
        # Additional winding configuration
        self.num_phases: Optional[int] = None  # typically 3
        self.turns_per_coil: Optional[int] = None
        self.wire_type: Optional[str] = None  # e.g., "round", "rectangular"
        self.parallel_paths: Optional[int] = None
        self.slot_fill_factor: Optional[float] = None  # 0-1
        
        # Material properties
        self.conductor_material: str = "Copper"
        self.resistivity: Optional[float] = None  # ohm-m at operating temp
    
    @property
    def stator_slots(self) -> Optional[int]:
        """Access parent's stator_slots for hierarchical calculations"""
        return self._parent.stator_slots if self._parent else None
        
    def __repr__(self):
        return f"StatorWindings(phases={self.num_phases}, turns={self.turns_per_coil})"
    
    def compute_mass(self) -> float:
        """Calculate stator windings mass (kg)"""
        # TODO: Implement based on wire dimensions and slot fill
        return 0.0
    
    def compute_cost(self) -> float:
        """Calculate stator windings cost ($)"""
        # TODO: Implement based on copper mass and cost per kg
        return 0.0
