"""
Stator Core Component
Laminated steel core of the stator
"""

from typing import Optional, Dict


class StatorCore:
    """
    Laminated steel core of stator
    Provides magnetic flux path
    """
    
    def __init__(self):
        # Parent reference for hierarchical access
        self._parent: Optional['Stator'] = None
        
        # Core parameters from Excel
        self.stator_core_stack_length: Optional[float] = None  # mm
        self.stator_core_outer_diameter: Optional[float] = None  # mm
        self.stator_core_slot_width: Optional[float] = None  # mm
        self.stator_core_slot_height: Optional[float] = None  # mm
        self.stator_core_slot_opening: Optional[float] = None  # mm
        self.stator_core_tooth_tip_height: Optional[float] = None  # mm
        self.stator_core_tooth_taper_height: Optional[float] = None  # mm
        
        # Additional core material properties
        self.material: Optional[str] = None  # e.g., "M19 Steel"
        self.lamination_thickness: Optional[float] = None  # mm
        self.stacking_factor: Optional[float] = None  # 0-1
        self.permeability: Optional[float] = None  # relative
        self.core_loss_coefficients: Optional[Dict] = None
    
    @property
    def stator_slots(self) -> Optional[int]:
        """Access parent's stator_slots for hierarchical calculations"""
        return self._parent.stator_slots if self._parent else None
    
    @property
    def stator_core_inner_diameter(self) -> Optional[float]:
        """Calculated: rotor_core_outer_diameter + 2 * magnet_thickness + 2 * motor_airgap"""
        if not self._parent or not self._parent._parent:
            return None
        
        motor = self._parent._parent  # Stator's parent is Motor
        rotor = motor.rotor
        
        if (rotor and rotor.core and rotor.core.rotor_core_outer_diameter and 
            rotor.magnet and rotor.magnet.magnet_thickness and motor.motor_airgap):
            return rotor.core.rotor_core_outer_diameter + (2 * rotor.magnet.magnet_thickness) + (2 * motor.motor_airgap)
        return None
        
    def __repr__(self):
        return f"StatorCore(OD={self.stator_core_outer_diameter}mm, stack_length={self.stator_core_stack_length}mm)"
    
    def compute_mass(self) -> float:
        """Calculate stator core mass (kg)"""
        # TODO: Implement based on geometry and material density
        return 0.0
    
    def compute_cost(self) -> float:
        """Calculate stator core cost ($)"""
        # TODO: Implement based on mass and material cost
        return 0.0
