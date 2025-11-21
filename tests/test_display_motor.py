"""
Test script for motor visualization
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from excel_importer import ExcelParameterImporter
from display_motor import display_motor


def main():
    """Test motor display with example design"""
    
    # Load example design
    excel_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        'projects',
        'example_design_1',
        'example_design_1_edrive_parameters.xlsx'
    )
    
    if not os.path.exists(excel_path):
        print(f"Error: Excel file not found at {excel_path}")
        return
    
    print(f"Loading parameters from: {excel_path}")
    
    # Import parameters
    importer = ExcelParameterImporter(excel_path)
    components = importer.import_all()
    
    if not components or not components.get('motor'):
        print("Error: Failed to load motor parameters")
        return
    
    motor = components['motor']
    
    print("\nMotor Parameters:")
    print(f"  Poles: {motor.motor_poles}")
    print(f"  Airgap: {motor.motor_airgap} mm")
    print(f"  Stator slots: {motor.stator.stator_slots}")
    print(f"  Stator OD: {motor.stator.core.stator_core_outer_diameter} mm")
    print(f"  Rotor OD: {motor.rotor.core.rotor_core_outer_diameter} mm")
    print(f"  Magnet thickness: {motor.rotor.magnet.magnet_thickness} mm")
    print(f"  Magnet arc: {motor.rotor.magnet.magnet_arc_percent}%")
    
    print("\nGenerating motor cross-section plot...")
    
    # Display motor
    display_motor(motor, show=True, save_path='motor_cross_section.png')


if __name__ == "__main__":
    main()
