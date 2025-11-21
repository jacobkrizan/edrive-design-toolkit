"""
Test Motor Mesher
Standalone test for motor mesh generation and gmsh visualization
"""

import sys
import os

# Add src directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from excel_importer import ExcelParameterImporter
from motor_mesher import mesh_motor

def test_motor_mesher():
    """Test motor mesh generation"""
    
    # Path to Excel file
    excel_file = os.path.join(
        os.path.dirname(__file__), 
        '..', 
        'projects', 
        'example_design_1',
        'example_design_1_edrive_parameters.xlsx'
    )
    
    print(f"Loading parameters from: {excel_file}")
    
    if not os.path.exists(excel_file):
        print(f"ERROR: Excel file not found: {excel_file}")
        return
    
    # Import parameters
    try:
        importer = ExcelParameterImporter(excel_file)
        components = importer.import_all()
        
        if not components or not components.get('motor'):
            print("ERROR: Failed to load motor from Excel file")
            return
        
        motor = components['motor']
        print(f"\nMotor Parameters:")
        print(f"  Poles: {motor.motor_poles}")
        print(f"  Airgap: {motor.motor_airgap} mm")
        if motor.stator:
            print(f"  Stator slots: {motor.stator.stator_slots}")
            if motor.stator.core:
                print(f"  Stator OD: {motor.stator.core.stator_core_outer_diameter} mm")
                print(f"  Stator ID: {motor.stator.core.stator_core_inner_diameter} mm")
                print(f"  Slot height: {motor.stator.core.stator_core_slot_height} mm")
                print(f"  Slot width: {motor.stator.core.stator_core_slot_width} mm")
        if motor.rotor:
            if motor.rotor.core:
                print(f"  Rotor OD: {motor.rotor.core.rotor_core_outer_diameter} mm")
                print(f"  Rotor ID: {motor.rotor.core.rotor_core_inner_diameter} mm")
            if motor.rotor.magnet:
                print(f"  Magnet thickness: {motor.rotor.magnet.magnet_thickness} mm")
                print(f"  Magnet arc: {motor.rotor.magnet.magnet_arc_percent}%")
        
        print("\n" + "="*60)
        print("Generating motor mesh...")
        print("="*60)
        
        # Generate mesh
        mesh_file = mesh_motor(motor, mesh_size=2.0, show_viewer=True)
        
        print(f"\nSUCCESS: Mesh generated and saved to: {mesh_file}")
        print("Gmsh viewer should have opened.")
        print("\nCheck the gmsh window to view the meshed motor geometry.")
        
    except Exception as e:
        print(f"\nERROR: Mesh generation failed")
        print(f"Exception type: {type(e).__name__}")
        print(f"Exception message: {str(e)}")
        
        # Print full traceback for debugging
        import traceback
        print("\nFull traceback:")
        traceback.print_exc()
        
        return

if __name__ == "__main__":
    print("="*60)
    print("Motor Mesher Test")
    print("="*60)
    test_motor_mesher()
