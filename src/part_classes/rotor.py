"""
Rotor Components
Rotor assembly and subcomponents
"""

from typing import Optional, List, Dict


class Rotor:
    """
    Rotor assembly
    Contains core, magnets, shaft, and endplates
    """
    
    def __init__(self):
        self.core: Optional['RotorCore'] = None
        self.magnets: List['Magnet'] = []
        self.shaft: Optional['Shaft'] = None
        self.endplates: List['Endplate'] = []
        
        # Rotor geometry
        self.outer_diameter: Optional[float] = None  # mm
        self.inner_diameter: Optional[float] = None  # mm
        self.stack_length: Optional[float] = None  # mm
        
    def __repr__(self):
        return f"Rotor(magnets={len(self.magnets)}, OD={self.outer_diameter}mm)"


class RotorCore:
    """
    Laminated steel core of rotor
    Back iron for magnetic flux return path
    """
    
    def __init__(self):
        # Core material properties
        self.material: Optional[str] = None  # e.g., "M19 Steel"
        self.lamination_thickness: Optional[float] = None  # mm
        self.stacking_factor: Optional[float] = None  # 0-1
        self.permeability: Optional[float] = None  # relative
        
        # Geometry
        self.back_iron_thickness: Optional[float] = None  # mm
        
    def __repr__(self):
        return f"RotorCore(material={self.material}, back_iron={self.back_iron_thickness}mm)"


class Magnet:
    """
    Permanent magnet
    Provides excitation field
    """
    
    def __init__(self):
        # Magnet properties
        self.material: Optional[str] = None  # e.g., "NdFeB N42"
        self.remanence: Optional[float] = None  # T (Br)
        self.coercivity: Optional[float] = None  # A/m (Hc)
        self.relative_permeability: Optional[float] = None
        self.max_operating_temp: Optional[float] = None  # °C
        
        # Geometry
        self.thickness: Optional[float] = None  # mm (radial)
        self.width: Optional[float] = None  # mm (tangential)
        self.length: Optional[float] = None  # mm (axial)
        self.pole_arc_ratio: Optional[float] = None  # 0-1
        
        # Mounting
        self.mounting_type: Optional[str] = None  # e.g., "surface", "interior", "inset"
        
    def __repr__(self):
        return f"Magnet(material={self.material}, Br={self.remanence}T, thickness={self.thickness}mm)"


class Shaft:
    """
    Rotating shaft
    Mechanical power transmission
    """
    
    def __init__(self):
        # Shaft properties
        self.material: Optional[str] = None  # e.g., "Steel 4140"
        self.diameter: Optional[float] = None  # mm
        self.length: Optional[float] = None  # mm
        self.yield_strength: Optional[float] = None  # MPa
        
        # Features
        self.keyway: bool = False
        self.spline: bool = False
        
    def __repr__(self):
        return f"Shaft(material={self.material}, diameter={self.diameter}mm)"


class Endplate:
    """
    Rotor endplate
    Axial retention of rotor components
    """
    
    def __init__(self):
        # Endplate properties
        self.material: Optional[str] = None  # e.g., "Aluminum", "Steel"
        self.thickness: Optional[float] = None  # mm
        self.outer_diameter: Optional[float] = None  # mm
        self.inner_diameter: Optional[float] = None  # mm
        
        # Mounting
        self.fastener_type: Optional[str] = None  # e.g., "bolt", "rivet"
        self.num_fasteners: Optional[int] = None
        
    def __repr__(self):
        return f"Endplate(material={self.material}, thickness={self.thickness}mm)"
