"""
Motor Component
Electric motor assembly containing stator and rotor
"""

from typing import Optional


class Motor:
    """
    Electric motor assembly
    Contains stator and rotor assemblies
    """
    
    def __init__(self):
        self.stator: Optional['Stator'] = None
        self.rotor: Optional['Rotor'] = None
        
        # Motor-level properties
        self.rated_power: Optional[float] = None  # W
        self.rated_speed: Optional[float] = None  # rpm
        self.rated_torque: Optional[float] = None  # Nm
        self.num_poles: Optional[int] = None
        self.stack_length: Optional[float] = None  # mm
        
    def __repr__(self):
        return f"Motor(poles={self.num_poles}, stator={self.stator}, rotor={self.rotor})"
