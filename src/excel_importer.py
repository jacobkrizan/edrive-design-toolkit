"""
Excel Parameter Importer
Reads e-drive component parameters from Excel files
"""

import pandas as pd
from typing import Dict, Any, Optional
import os

from part_classes import (Motor, Stator, StatorWindings, SlotLiner, StatorCore,
                          Rotor, RotorCore, Magnet, Shaft, Endplate,
                          Battery, Inverter, Gearbox)


class ExcelParameterImporter:
    """
    Import e-drive parameters from Excel file
    Expected Excel structure with sheets: Motor, Stator, Rotor, Battery, Inverter, Gearbox
    """
    
    def __init__(self, excel_path: str):
        """
        Initialize importer with path to Excel file
        
        Args:
            excel_path: Path to Excel file containing parameters
        """
        self.excel_path = excel_path
        self.data: Dict[str, pd.DataFrame] = {}
        
    def load_excel(self) -> bool:
        """
        Load all sheets from Excel file
        
        Returns:
            True if successful, False otherwise
        """
        try:
            xl_file = pd.ExcelFile(self.excel_path)
            for sheet_name in xl_file.sheet_names:
                self.data[sheet_name] = pd.read_excel(self.excel_path, sheet_name=sheet_name)
            return True
        except Exception as e:
            print(f"Error loading Excel file: {e}")
            return False
    
    def _read_parameter_sheet(self, sheet_name: str) -> Dict[str, Any]:
        """
        Read a parameter sheet with Parameter, Units, Value columns
        
        Args:
            sheet_name: Name of the sheet to read (case-insensitive)
            
        Returns:
            Dictionary of parameter: value pairs
        """
        # Try case-insensitive sheet name matching
        actual_sheet = None
        for key in self.data.keys():
            if str(key).lower() == sheet_name.lower():
                actual_sheet = key
                break
        
        if actual_sheet is None:
            return {}
        
        df = self.data[actual_sheet]
        params = {}
        
        # Assume format: Parameter | Units | Value (column order may vary)
        for _, row in df.iterrows():
            param_col = row.get('Parameter')
            value_col = row.get('Value')
            
            if pd.notna(param_col) and pd.notna(value_col):
                param_name = str(param_col).strip()
                value = value_col
                
                # Try to convert to appropriate type
                try:
                    if isinstance(value, str):
                        value = value.strip()
                        # Try numeric conversion
                        try:
                            if '.' in value:
                                value = float(value)
                            else:
                                value = int(value)
                        except:
                            pass  # Keep as string
                except:
                    pass
                
                params[param_name] = value
        
        return params
    
    def import_motor(self) -> Optional[Motor]:
        """Import motor parameters"""
        motor = Motor()
        stator = motor.stator
        stator_core = stator.core
        stator_windings = stator.windings
        slot_liner = stator.slot_liner
        rotor = motor.rotor
        rotor_core = rotor.core
        magnet = rotor.magnet
        params = self._read_parameter_sheet('Motor')
        
        # Assign parameters based on their prefix
        for param_name, value in params.items():
            # Motor-level parameters (motor_*)
            if param_name.startswith('motor_'):
                if hasattr(motor, param_name):
                    setattr(motor, param_name, value)
            
            # Stator-level parameters (stator_slots)
            elif param_name == 'stator_slots':
                if hasattr(stator, param_name):
                    setattr(stator, param_name, value)
            
            # Stator core parameters (stator_core_*)
            elif param_name.startswith('stator_core_'):
                if hasattr(stator_core, param_name):
                    setattr(stator_core, param_name, value)
            
            # Stator winding parameters (stator_winding_*)
            elif param_name.startswith('stator_winding_'):
                if hasattr(stator_windings, param_name):
                    setattr(stator_windings, param_name, value)
            
            # Slot liner parameters (slot_liner_*)
            elif param_name.startswith('slot_liner_'):
                if hasattr(slot_liner, param_name):
                    setattr(slot_liner, param_name, value)
            
            # Rotor core parameters (rotor_core_*)
            elif param_name.startswith('rotor_core_'):
                if hasattr(rotor_core, param_name):
                    setattr(rotor_core, param_name, value)
            
            # Magnet parameters (magnet_*)
            elif param_name.startswith('magnet_'):
                if hasattr(magnet, param_name):
                    setattr(magnet, param_name, value)
        
        return motor
    
    def import_stator(self) -> Optional[Stator]:
        """Import stator parameters"""
        stator = Stator()
        params = self._read_parameter_sheet('Stator')
        
        # Stator geometry
        if 'outer_diameter' in params:
            stator.outer_diameter = params['outer_diameter']
        if 'inner_diameter' in params:
            stator.inner_diameter = params['inner_diameter']
        if 'stack_length' in params:
            stator.stack_length = params['stack_length']
        if 'num_slots' in params:
            stator.num_slots = params['num_slots']
        
        # Windings
        stator.windings = StatorWindings()
        if 'num_phases' in params:
            stator.windings.num_phases = params['num_phases']
        if 'turns_per_coil' in params:
            stator.windings.turns_per_coil = params['turns_per_coil']
        if 'wire_diameter' in params:
            stator.windings.wire_diameter = params['wire_diameter']
        if 'parallel_paths' in params:
            stator.windings.parallel_paths = params['parallel_paths']
        
        # Slot liner
        stator.slot_liner = SlotLiner()
        if 'liner_material' in params:
            stator.slot_liner.material = params['liner_material']
        if 'liner_thickness' in params:
            stator.slot_liner.thickness = params['liner_thickness']
        
        # Core
        stator.core = StatorCore()
        if 'core_material' in params:
            stator.core.material = params['core_material']
        if 'lamination_thickness' in params:
            stator.core.lamination_thickness = params['lamination_thickness']
        if 'stacking_factor' in params:
            stator.core.stacking_factor = params['stacking_factor']
        
        return stator
    
    def import_rotor(self) -> Optional[Rotor]:
        """Import rotor parameters"""
        rotor = Rotor()
        params = self._read_parameter_sheet('Rotor')
        
        # Rotor geometry
        if 'outer_diameter' in params:
            rotor.outer_diameter = params['outer_diameter']
        if 'inner_diameter' in params:
            rotor.inner_diameter = params['inner_diameter']
        if 'stack_length' in params:
            rotor.stack_length = params['stack_length']
        
        # Core
        rotor.core = RotorCore()
        if 'core_material' in params:
            rotor.core.material = params['core_material']
        if 'back_iron_thickness' in params:
            rotor.core.back_iron_thickness = params['back_iron_thickness']
        
        # Magnets
        if 'num_magnets' in params:
            num_magnets = params['num_magnets']
            for i in range(int(num_magnets)):
                magnet = Magnet()
                if 'magnet_material' in params:
                    magnet.material = params['magnet_material']
                if 'magnet_remanence' in params:
                    magnet.remanence = params['magnet_remanence']
                if 'magnet_coercivity' in params:
                    magnet.coercivity = params['magnet_coercivity']
                if 'magnet_thickness' in params:
                    magnet.thickness = params['magnet_thickness']
                if 'magnet_width' in params:
                    magnet.width = params['magnet_width']
                rotor.magnets.append(magnet)
        
        # Shaft
        rotor.shaft = Shaft()
        if 'shaft_diameter' in params:
            rotor.shaft.diameter = params['shaft_diameter']
        if 'shaft_material' in params:
            rotor.shaft.material = params['shaft_material']
        
        return rotor
    
    def import_battery(self) -> Optional[Battery]:
        """Import battery parameters"""
        battery = Battery()
        params = self._read_parameter_sheet('Battery')
        
        # Assign parameters based on their prefix
        for param_name, value in params.items():
            if param_name.startswith('battery_'):
                if hasattr(battery, param_name):
                    setattr(battery, param_name, value)
        
        return battery
    
    def import_inverter(self) -> Optional[Inverter]:
        """Import inverter parameters"""
        inverter = Inverter()
        params = self._read_parameter_sheet('Inverter')
        
        # Assign parameters based on their prefix
        for param_name, value in params.items():
            if param_name.startswith('inverter_'):
                if hasattr(inverter, param_name):
                    setattr(inverter, param_name, value)
        
        return inverter
    
    def import_gearbox(self) -> Optional[Gearbox]:
        """Import gearbox parameters"""
        gearbox = Gearbox()
        params = self._read_parameter_sheet('Gearbox')
        
        # Assign parameters based on their prefix
        for param_name, value in params.items():
            if param_name.startswith('gearbox_'):
                if hasattr(gearbox, param_name):
                    setattr(gearbox, param_name, value)
        
        return gearbox
    
    def import_all(self) -> Dict[str, Any]:
        """
        Import all components from Excel file
        
        Returns:
            Dictionary with all components
        """
        if not self.load_excel():
            return {}
        
        # Import motor (which includes stator and rotor from motor sheet)
        motor = self.import_motor()
        
        components = {
            'motor': motor,
            'stator': motor.stator if motor else None,
            'rotor': motor.rotor if motor else None,
            'battery': self.import_battery(),
            'inverter': self.import_inverter(),
            'gearbox': self.import_gearbox(),
        }
        
        return components


def load_project_from_excel(project_folder: str) -> Optional[Dict[str, Any]]:
    """
    Load a project from an Excel file in the projects folder
    
    Args:
        project_folder: Name of the project folder containing Excel file
        
    Returns:
        Dictionary of components or None if loading fails
    """
    # Find Excel file in project folder
    base_path = os.path.join(os.path.dirname(__file__), '..', 'projects', project_folder)
    
    excel_files = [f for f in os.listdir(base_path) if f.endswith('.xlsx')]
    
    if not excel_files:
        print(f"No Excel files found in {base_path}")
        return None
    
    excel_path = os.path.join(base_path, excel_files[0])
    
    importer = ExcelParameterImporter(excel_path)
    return importer.import_all()
