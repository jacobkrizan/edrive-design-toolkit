"""
Stator Component
Stator assembly - parent component for windings, slot liner, and core
"""

from typing import Optional
from .stator_core import StatorCore
from .stator_windings import StatorWindings
from .slot_liner import SlotLiner


class Stator:
    """
    Stator assembly
    Contains windings, slot liner, and core
    """
    
    def __init__(self):
        # Parent reference for hierarchical access
        self._parent: Optional['Motor'] = None
        
        # Stator parameters from Excel
        self.stator_slots: Optional[int] = None  # Number of slots
        
        # Sub-components - create instances so they can access parent
        self.core: Optional[StatorCore] = StatorCore()
        self.windings: Optional[StatorWindings] = StatorWindings()
        self.slot_liner: Optional[SlotLiner] = SlotLiner()
        
        # Set parent reference for hierarchical access
        self.core._parent = self
        self.windings._parent = self
        self.slot_liner._parent = self
        
    def __repr__(self):
        return f"Stator(slots={self.stator_slots})"
    
    def compute_mass(self) -> float:
        """Calculate total stator mass (kg)"""
        total_mass = 0.0
        if self.core:
            total_mass += self.core.compute_mass()
        if self.windings:
            total_mass += self.windings.compute_mass()
        if self.slot_liner:
            total_mass += self.slot_liner.compute_mass()
        return total_mass
    
    def compute_cost(self) -> float:
        """Calculate total stator cost ($)"""
        total_cost = 0.0
        if self.core:
            total_cost += self.core.compute_cost()
        if self.windings:
            total_cost += self.windings.compute_cost()
        if self.slot_liner:
            total_cost += self.slot_liner.compute_cost()
        return total_cost
