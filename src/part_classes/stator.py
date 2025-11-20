"""
Stator Components
Stator assembly and subcomponents
"""

from typing import Optional, Dict


class Stator:
    """
    Stator assembly
    Contains windings, slot liner, and core
    """
    
    def __init__(self):
        self.windings: Optional['StatorWindings'] = None
        self.slot_liner: Optional['SlotLiner'] = None
        self.core: Optional['StatorCore'] = None
        
        # Stator geometry
        self.outer_diameter: Optional[float] = None  # mm
        self.inner_diameter: Optional[float] = None  # mm
        self.stack_length: Optional[float] = None  # mm
        self.num_slots: Optional[int] = None
        
    def __repr__(self):
        return f"Stator(slots={self.num_slots}, OD={self.outer_diameter}mm, ID={self.inner_diameter}mm)"


class StatorWindings:
    """
    Stator winding configuration
    Copper coils in stator slots
    """
    
    def __init__(self):
        # Winding configuration
        self.num_phases: Optional[int] = None  # typically 3
        self.turns_per_coil: Optional[int] = None
        self.wire_diameter: Optional[float] = None  # mm
        self.wire_type: Optional[str] = None  # e.g., "round", "rectangular"
        self.parallel_paths: Optional[int] = None
        self.slot_fill_factor: Optional[float] = None  # 0-1
        
        # Material properties
        self.conductor_material: str = "Copper"
        self.resistivity: Optional[float] = None  # ohm-m at operating temp
        
    def __repr__(self):
        return f"StatorWindings(phases={self.num_phases}, turns={self.turns_per_coil})"


class SlotLiner:
    """
    Electrical insulation in stator slots
    Separates windings from core
    """
    
    def __init__(self):
        # Liner properties
        self.material: Optional[str] = None  # e.g., "Nomex", "Mylar"
        self.thickness: Optional[float] = None  # mm
        self.dielectric_strength: Optional[float] = None  # kV/mm
        self.thermal_conductivity: Optional[float] = None  # W/m-K
        
    def __repr__(self):
        return f"SlotLiner(material={self.material}, thickness={self.thickness}mm)"


class StatorCore:
    """
    Laminated steel core of stator
    Provides magnetic flux path
    """
    
    def __init__(self):
        # Core material properties
        self.material: Optional[str] = None  # e.g., "M19 Steel"
        self.lamination_thickness: Optional[float] = None  # mm
        self.stacking_factor: Optional[float] = None  # 0-1
        self.permeability: Optional[float] = None  # relative
        self.core_loss_coefficients: Optional[Dict] = None
        
        # Geometry
        self.tooth_width: Optional[float] = None  # mm
        self.yoke_thickness: Optional[float] = None  # mm
        
    def __repr__(self):
        return f"StatorCore(material={self.material}, lamination={self.lamination_thickness}mm)"
