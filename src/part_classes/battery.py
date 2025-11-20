"""
Battery Component
Energy storage for electric drive system
"""

from typing import Optional


class Battery:
    """
    Battery pack
    Energy storage for the electric drive
    """
    
    def __init__(self):
        # Battery properties
        self.voltage_nominal: Optional[float] = None  # V
        self.capacity: Optional[float] = None  # Ah
        self.energy: Optional[float] = None  # Wh
        self.chemistry: Optional[str] = None  # e.g., "NMC", "LFP"
        self.cell_configuration: Optional[str] = None  # e.g., "96s2p"
        
    def __repr__(self):
        return f"Battery({self.voltage_nominal}V, {self.capacity}Ah, {self.chemistry})"
