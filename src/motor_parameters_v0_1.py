"""
Motor Parameters Module
Defines all design parameters and performance metrics for permanent magnet motors
Loads parameters from Excel file for dynamic updates
"""

import numpy as np
import pandas as pd
import os
from dataclasses import dataclass, field
from typing import Dict, Any, Optional


def load_parameters_from_excel(excel_path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load motor parameters from Excel file
    
    Args:
        excel_path: Path to Excel file. If None, uses default project path.
        
    Returns:
        Dictionary of parameter values
    """
    if excel_path is None:
        # Default to example_design_1
        script_dir = os.path.dirname(os.path.abspath(__file__))
        repo_root = os.path.dirname(script_dir)
        excel_path = os.path.join(repo_root, 'projects', 'example_design_1', 
                                  'example_design_1_edrive_parameters.xlsx')
    
    if not os.path.exists(excel_path):
        print(f"Warning: Excel file not found at {excel_path}. Using default values.")
        return {}
    
    try:
        # Read motor sheet
        motor_df = pd.read_excel(excel_path, sheet_name='motor')
        motor_params = {}
        
        for _, row in motor_df.iterrows():
            param_name = row.get('Parameter')
            value = row.get('Value')
            
            if pd.notna(param_name) and pd.notna(value):
                param_name = str(param_name).strip()
                motor_params[param_name] = value
        
        # Read battery sheet for voltage
        try:
            battery_df = pd.read_excel(excel_path, sheet_name='battery')
            for _, row in battery_df.iterrows():
                param_name = row.get('Parameter')
                value = row.get('Value')
                
                if pd.notna(param_name) and pd.notna(value):
                    param_name = str(param_name).strip()
                    motor_params[param_name] = value
        except:
            pass  # Battery sheet optional
        
        return motor_params
        
    except Exception as e:
        print(f"Warning: Error loading Excel file: {e}. Using default values.")
        return {}


@dataclass
class MotorParameters:
    """Container for all motor design parameters - loads from Excel by default"""
    
    # Excel file path (set to None to use default project path)
    excel_path: Optional[str] = None
    
    # Basic Configuration
    num_poles: int = 8
    num_slots: int = 48
    
    # Geometric Parameters (mm)
    stator_outer_diameter: float = 200.0
    stator_inner_diameter: float = 148.0  # Calculated: rotor_OD + 2*magnet_thickness + 2*airgap
    rotor_outer_diameter: float = 138.0  # Rotor core outer diameter
    rotor_inner_diameter: float = 50.0
    airgap_length: float = 1.0  # Gap between magnet outer surface and stator bore
    stack_length: float = 100.0
    
    # Slot Geometry
    slot_width: float = 4.0  # Tangential width of slot
    slot_height: float = 12.0  # Radial depth of slot
    slot_opening: float = 1.0  # Narrow opening at airgap
    tooth_tip_height: float = 0.5
    tooth_taper_height: float = 0.5
    tooth_width: float = 5.0
    
    # Magnet Geometry (Surface Mount)
    magnet_thickness: float = 4.0
    magnet_arc_ratio: float = 0.80  # Magnet arc / pole pitch (80%)
    
    # Winding Configuration
    turns_per_coil: int = 20
    parallel_paths: int = 2
    winding_layers: int = 2
    coil_pitch: int = 5  # slots
    
    # Electrical Parameters
    rated_current: float = 100.0  # Arms
    rated_voltage: float = 100.0  # Vdc (battery voltage)
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
        """Calculate derived parameters and optionally load from Excel"""
        # Load from Excel if values match defaults (indicating no explicit override)
        excel_params = load_parameters_from_excel(self.excel_path)
        
        if excel_params:
            # Map Excel parameter names to class attributes
            param_mapping = {
                'motor_poles': 'num_poles',
                'motor_airgap': 'airgap_length',
                'stator_slots': 'num_slots',
                'stator_core_stack_length': 'stack_length',
                'stator_core_outer_diameter': 'stator_outer_diameter',
                'stator_core_slot_width': 'slot_width',
                'stator_core_slot_height': 'slot_height',
                'stator_core_slot_opening': 'slot_opening',
                'stator_core_tooth_tip_height': 'tooth_tip_height',
                'stator_core_tooth_taper_height': 'tooth_taper_height',
                'rotor_core_outer_diameter': 'rotor_outer_diameter',
                'rotor_core_inner_diameter': 'rotor_inner_diameter',
                'magnet_thickness': 'magnet_thickness',
                'magnet_arc_percent': 'magnet_arc_ratio',  # Will convert % to ratio
                'battery_voltage': 'rated_voltage',
            }
            
            for excel_name, attr_name in param_mapping.items():
                if excel_name in excel_params:
                    value = excel_params[excel_name]
                    
                    # Special handling for percentage to ratio conversion
                    if excel_name == 'magnet_arc_percent':
                        value = value / 100.0  # Convert 80% to 0.80
                    
                    # Convert to int for countable parameters
                    if attr_name in ['num_poles', 'num_slots']:
                        value = int(value)
                    
                    setattr(self, attr_name, value)
            
            # Calculate stator inner diameter from rotor dimensions
            magnet_outer_r = self.rotor_outer_diameter / 2 + self.magnet_thickness
            self.stator_inner_diameter = (magnet_outer_r + self.airgap_length) * 2
        
        # Calculate derived parameters
        self.pole_pitch = np.pi * self.rotor_outer_diameter / self.num_poles
        self.slot_pitch = np.pi * self.stator_inner_diameter / self.num_slots
        self.slots_per_pole_per_phase = self.num_slots / (self.num_poles * 3)
        self.electrical_frequency = self.rated_speed * self.num_poles / 120.0
        
        # Derived slot parameter for backward compatibility
        self.slot_depth = self.slot_height  # slot_depth is same as slot_height (radial dimension)
        
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
