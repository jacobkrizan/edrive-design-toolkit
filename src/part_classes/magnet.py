"""
Magnet Component
Permanent magnets for rotor excitation
"""

from typing import Optional


class Magnet:
    """
    Permanent magnet
    Provides excitation field
    """
    
    def __init__(self):
        # Parent reference for hierarchical access
        self._parent: Optional['Rotor'] = None
        
        # Magnet parameters from Excel
        self.magnet_thickness: Optional[float] = None  # mm (radial)
        self.magnet_arc_percent: Optional[float] = None  # % pole arc coverage
        
        # Additional magnet properties
        self.material: Optional[str] = None  # e.g., "NdFeB N42"
        self.remanence: Optional[float] = None  # T (Br)
        self.coercivity: Optional[float] = None  # A/m (Hc)
        self.relative_permeability: Optional[float] = None
        self.max_operating_temp: Optional[float] = None  # °C
        
        # Additional geometry
        self.width: Optional[float] = None  # mm (tangential)
        self.length: Optional[float] = None  # mm (axial)
        
        # Mounting
        self.mounting_type: Optional[str] = None  # e.g., "surface", "interior", "inset"
        
    def __repr__(self):
        return f"Magnet(thickness={self.magnet_thickness}mm, arc={self.magnet_arc_percent}%)"
    
    def compute_mass(self) -> float:
        """Calculate magnet mass (kg) for all poles"""
        # TODO: Implement based on magnet volume and material density
        return 0.0
    
    def compute_cost(self) -> float:
        """Calculate magnet cost ($) for all poles"""
        # TODO: Implement based on mass and rare earth material cost
        return 0.0
