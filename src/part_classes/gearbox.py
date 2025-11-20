"""
Gearbox Component
Mechanical gearbox for speed reduction
"""

from typing import Optional


class Gearbox:
    """
    Mechanical gearbox
    Provides speed reduction and torque multiplication
    """
    
    def __init__(self):
        # Gearbox properties
        self.gear_ratio: Optional[float] = None
        self.efficiency: Optional[float] = None  # 0-1
        self.max_input_speed: Optional[float] = None  # rpm
        self.max_torque: Optional[float] = None  # Nm
        self.type: Optional[str] = None  # e.g., "planetary", "helical"
        
    def __repr__(self):
        return f"Gearbox(ratio={self.gear_ratio}, type={self.type})"
