"""
Motor Parameters Module
Defines all design parameters and performance metrics for permanent magnet motors
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, Any


@dataclass
class MotorParameters:
    """Container for all motor design parameters"""
    
    # Basic Configuration
    num_poles: int = 8
    num_slots: int = 48
    
    # Geometric Parameters (mm)
    stator_outer_diameter: float = 200.0
    stator_inner_diameter: float = 120.0
    rotor_outer_diameter: float = 118.0
    rotor_inner_diameter: float = 40.0
    airgap_length: float = 1.0
    stack_length: float = 100.0
    
    # Slot Geometry
    slot_depth: float = 30.0
    slot_opening: float = 3.0
    tooth_width: float = 5.0
    
    # Magnet Geometry (Surface Mount)
    magnet_thickness: float = 5.0
    magnet_arc_ratio: float = 0.85  # Magnet arc / pole pitch
    
    # Winding Configuration
    turns_per_coil: int = 20
    parallel_paths: int = 2
    winding_layers: int = 2
    coil_pitch: int = 5  # slots
    
    # Electrical Parameters
    rated_current: float = 100.0  # Arms
    rated_voltage: float = 400.0  # Vrms (line-line)
    rated_speed: float = 3000.0  # rpm
    frequency: float = 200.0  # Hz
    
    # Material Properties
    magnet_br: float = 1.2  # Residual flux density (T)
    magnet_hc: float = -900000.0  # Coercivity (A/m)
    magnet_mu_r: float = 1.05  # Relative permeability
    
    steel_mu_r: float = 2000.0  # Relative permeability of electrical steel
    steel_bsat: float = 1.8  # Saturation flux density (T)
    
    copper_resistivity: float = 1.72e-8  # Ohm-m at 20°C
    fill_factor: float = 0.45  # Slot fill factor
    
    # Thermal Parameters
    ambient_temp: float = 40.0  # °C
    max_winding_temp: float = 155.0  # °C (Class F insulation)
    
    def __post_init__(self):
        """Calculate derived parameters"""
        self.pole_pitch = np.pi * self.rotor_outer_diameter / self.num_poles
        self.slot_pitch = np.pi * self.stator_inner_diameter / self.num_slots
        self.slots_per_pole_per_phase = self.num_slots / (self.num_poles * 3)
        self.electrical_frequency = self.rated_speed * self.num_poles / 120.0
        
    def to_dict(self) -> Dict[str, Any]:
        """Convert parameters to dictionary"""
        return {k: v for k, v in self.__dict__.items()}


@dataclass
class PerformanceMetrics:
    """Container for calculated motor performance metrics"""
    
    # Electromagnetic Metrics
    torque: float = 0.0  # Nm
    power: float = 0.0  # kW
    back_emf: float = 0.0  # Vrms
    flux_linkage: float = 0.0  # Wb-turns
    
    # Flux Density
    airgap_flux_density_peak: float = 0.0  # T
    tooth_flux_density: float = 0.0  # T
    yoke_flux_density: float = 0.0  # T
    
    # Efficiency Metrics
    copper_loss: float = 0.0  # W
    iron_loss: float = 0.0  # W
    efficiency: float = 0.0  # %
    power_factor: float = 0.0
    
    # Torque Quality
    torque_ripple: float = 0.0  # %
    cogging_torque: float = 0.0  # Nm
    
    # Thermal
    current_density: float = 0.0  # A/mm²
    winding_resistance: float = 0.0  # Ohm
    temperature_rise: float = 0.0  # °C
    
    # Material Utilization
    active_mass: float = 0.0  # kg
    torque_density: float = 0.0  # Nm/kg
    power_density: float = 0.0  # kW/kg
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert metrics to dictionary"""
        return {k: v for k, v in self.__dict__.items()}
    
    def format_output(self) -> str:
        """Format metrics for display"""
        output = "=== PERFORMANCE METRICS ===\n\n"
        output += f"Torque:           {self.torque:.2f} Nm\n"
        output += f"Power:            {self.power:.2f} kW\n"
        output += f"Back EMF:         {self.back_emf:.1f} Vrms\n"
        output += f"Efficiency:       {self.efficiency:.1f} %\n"
        output += f"Power Factor:     {self.power_factor:.3f}\n\n"
        
        output += "--- Flux Density ---\n"
        output += f"Airgap (peak):    {self.airgap_flux_density_peak:.3f} T\n"
        output += f"Tooth:            {self.tooth_flux_density:.3f} T\n"
        output += f"Yoke:             {self.yoke_flux_density:.3f} T\n\n"
        
        output += "--- Losses ---\n"
        output += f"Copper Loss:      {self.copper_loss:.1f} W\n"
        output += f"Iron Loss:        {self.iron_loss:.1f} W\n\n"
        
        output += "--- Torque Quality ---\n"
        output += f"Torque Ripple:    {self.torque_ripple:.1f} %\n"
        output += f"Cogging Torque:   {self.cogging_torque:.3f} Nm\n\n"
        
        output += "--- Thermal ---\n"
        output += f"Current Density:  {self.current_density:.2f} A/mm²\n"
        output += f"Winding Resist.:  {self.winding_resistance:.4f} Ohm\n"
        output += f"Temp. Rise:       {self.temperature_rise:.1f} °C\n\n"
        
        output += "--- Utilization ---\n"
        output += f"Torque Density:   {self.torque_density:.2f} Nm/kg\n"
        output += f"Power Density:    {self.power_density:.2f} kW/kg\n"
        
        return output
