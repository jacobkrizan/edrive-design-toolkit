"""
Rotor Component
Rotor assembly - parent component for core, magnets, shaft, and endplates
"""

from typing import Optional, List
from .rotor_core import RotorCore
from .magnet import Magnet
from .shaft import Shaft
from .endplate import Endplate


class Rotor:
    """
    Rotor assembly
    Contains core, magnets, shaft, and endplates
    """
    
    def __init__(self):
        # Parent reference for hierarchical access
        self._parent: Optional['Motor'] = None
        
        # Rotor parameters from Excel
        # (currently no rotor-level parameters, all belong to sub-components)
        
        # Sub-components - create instances
        self.core: RotorCore = RotorCore()
        self.magnet: Magnet = Magnet()  # Single magnet instance (parameters apply to all poles)
        self.shaft: Optional[Shaft] = Shaft()
        self.endplates: List[Endplate] = []
        
        # Set parent references for hierarchical access
        self.core._parent = self
        self.magnet._parent = self
        if self.shaft:
            self.shaft._parent = self
        
    def __repr__(self):
        return f"Rotor(core={self.core}, magnet={self.magnet})"
    
    def compute_mass(self) -> float:
        """Calculate total rotor mass (kg)"""
        total_mass = 0.0
        if self.core:
            total_mass += self.core.compute_mass()
        if self.magnet:
            total_mass += self.magnet.compute_mass()
        if self.shaft:
            total_mass += self.shaft.compute_mass()
        for endplate in self.endplates:
            total_mass += endplate.compute_mass()
        return total_mass
    
    def compute_cost(self) -> float:
        """Calculate total rotor cost ($)"""
        total_cost = 0.0
        if self.core:
            total_cost += self.core.compute_cost()
        if self.magnet:
            total_cost += self.magnet.compute_cost()
        if self.shaft:
            total_cost += self.shaft.compute_cost()
        for endplate in self.endplates:
            total_cost += endplate.compute_cost()
        return total_cost
