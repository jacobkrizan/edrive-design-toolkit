"""
Drive Unit Component
Integrated motor, inverter, and gearbox assembly
"""

from typing import Optional
from .motor import Motor
from .inverter import Inverter
from .gearbox import Gearbox


class DriveUnit:
    """
    Drive unit assembly
    Contains motor, inverter, and gearbox
    """
    
    def __init__(self):
        # Parent reference for hierarchical access
        self._parent: Optional['Powertrain'] = None
        
        # Sub-components - create instances
        self.motor: Motor = Motor()
        self.inverter: Inverter = Inverter()
        self.gearbox: Gearbox = Gearbox()
        
        # Set parent references
        self.motor._parent = self
        self.inverter._parent = self
        self.gearbox._parent = self
        
    def __repr__(self):
        return f"DriveUnit(motor={self.motor}, inverter={self.inverter}, gearbox={self.gearbox})"
    
    def compute_mass(self) -> float:
        """Calculate total drive unit mass (kg)"""
        total_mass = 0.0
        if self.motor:
            total_mass += self.motor.compute_mass()
        if self.inverter:
            total_mass += self.inverter.compute_mass()
        if self.gearbox:
            total_mass += self.gearbox.compute_mass()
        return total_mass
    
    def compute_cost(self) -> float:
        """Calculate total drive unit cost ($)"""
        total_cost = 0.0
        if self.motor:
            total_cost += self.motor.compute_cost()
        if self.inverter:
            total_cost += self.inverter.compute_cost()
        if self.gearbox:
            total_cost += self.gearbox.compute_cost()
        return total_cost
