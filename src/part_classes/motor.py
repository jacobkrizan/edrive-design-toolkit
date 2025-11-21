"""
Motor Component
Electric motor assembly containing stator and rotor
"""

from typing import Optional
from .stator import Stator
from .rotor import Rotor


class Motor:
    """
    Electric motor assembly
    Contains stator and rotor assemblies
    """
    
    def __init__(self):
        # Parent reference for hierarchical access
        self._parent: Optional['DriveUnit'] = None
        
        # Motor-level properties from Excel
        self.motor_poles: Optional[int] = None  # Number of poles
        self.motor_airgap: Optional[float] = None  # mm
        
        # Sub-components - create instances
        self.stator: Stator = Stator()
        self.rotor: Rotor = Rotor()
        
        # Set parent references for hierarchical calculations
        self.stator._parent = self
        self.rotor._parent = self
        
    def __repr__(self):
        return f"Motor(poles={self.motor_poles}, airgap={self.motor_airgap}mm)"
    
    def compute_mass(self) -> float:
        """Calculate total motor mass (kg)"""
        total_mass = 0.0
        if self.stator:
            total_mass += self.stator.compute_mass()
        if self.rotor:
            total_mass += self.rotor.compute_mass()
        return total_mass
    
    def compute_cost(self) -> float:
        """Calculate total motor cost ($)"""
        total_cost = 0.0
        if self.stator:
            total_cost += self.stator.compute_cost()
        if self.rotor:
            total_cost += self.rotor.compute_cost()
        return total_cost
