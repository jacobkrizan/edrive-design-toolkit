"""
Powertrain Component
Complete electric powertrain with drive units and battery
"""

from typing import Optional
from .drive_unit import DriveUnit
from .battery import Battery


class Powertrain:
    """
    Complete electric powertrain
    Contains primary drive unit, optional secondary drive unit, and battery
    """
    
    def __init__(self):
        # Sub-components - create instances
        self.primary_drive_unit: DriveUnit = DriveUnit()
        self.secondary_drive_unit: Optional[DriveUnit] = None  # For dual-motor configurations
        self.battery: Battery = Battery()
        
        # Set parent references
        self.primary_drive_unit._parent = self
        self.battery._parent = self
        
    def __repr__(self):
        if self.secondary_drive_unit:
            return f"Powertrain(primary={self.primary_drive_unit}, secondary={self.secondary_drive_unit}, battery={self.battery})"
        return f"Powertrain(primary={self.primary_drive_unit}, battery={self.battery})"
    
    def compute_mass(self) -> float:
        """Calculate total powertrain mass (kg)"""
        total_mass = 0.0
        if self.primary_drive_unit:
            total_mass += self.primary_drive_unit.compute_mass()
        if self.secondary_drive_unit:
            total_mass += self.secondary_drive_unit.compute_mass()
        if self.battery:
            total_mass += self.battery.compute_mass()
        return total_mass
    
    def compute_cost(self) -> float:
        """Calculate total powertrain cost ($)"""
        total_cost = 0.0
        if self.primary_drive_unit:
            total_cost += self.primary_drive_unit.compute_cost()
        if self.secondary_drive_unit:
            total_cost += self.secondary_drive_unit.compute_cost()
        if self.battery:
            total_cost += self.battery.compute_cost()
        return total_cost
