"""
Test script for magnetostatic FEA solver
Tests the complete workflow: mesh generation -> FEA solve -> visualization
"""

import sys
import os

# Add parent directory and src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from src.excel_importer import ExcelParameterImporter
from src.motor_mesher import mesh_motor
from src.fea_solver_magnetostatic import solve_magnetostatic
import traceback


def test_magnetostatic_solver():
    """Test complete magnetostatic FEA workflow"""
    
    print("=" * 60)
    print("Magnetostatic FEA Solver Test")
    print("=" * 60)
    
    # Load motor parameters from Excel
    excel_file = os.path.join(
        os.path.dirname(__file__), 
        '..', 
        'projects', 
        'example_design_1', 
        'example_design_1_edrive_parameters.xlsx'
    )
    
    if not os.path.exists(excel_file):
        print(f"ERROR: Excel file not found: {excel_file}")
        return
    
    print(f"Loading parameters from: {excel_file}")
    
    importer = ExcelParameterImporter(excel_file)
    components = importer.import_all()
    motor = components['motor']
    
    # Print motor parameters
    print("\nMotor Parameters:")
    print(f"  Poles: {motor.motor_poles}")
    print(f"  Airgap: {motor.motor_airgap} mm")
    print(f"  Stator slots: {motor.stator.stator_slots}")
    
    if motor.stator and motor.stator.core:
        print(f"  Stator OD: {motor.stator.core.stator_core_outer_diameter} mm")
        print(f"  Stator ID: {motor.stator.core.stator_core_inner_diameter} mm")
        print(f"  Slot height: {motor.stator.core.stator_core_slot_height} mm")
        print(f"  Slot width: {motor.stator.core.stator_core_slot_width} mm")
    
    if motor.rotor and motor.rotor.core:
        print(f"  Rotor OD: {motor.rotor.core.rotor_core_outer_diameter} mm")
        print(f"  Rotor ID: {motor.rotor.core.rotor_core_inner_diameter} mm")
    
    if motor.rotor and motor.rotor.magnet:
        print(f"  Magnet thickness: {motor.rotor.magnet.magnet_thickness} mm")
        print(f"  Magnet arc: {motor.rotor.magnet.magnet_arc_percent}%")
    
    # Step 1: Generate mesh
    print("\n" + "=" * 60)
    print("Step 1: Generating motor mesh...")
    print("=" * 60)
    
    try:
        mesh_file = mesh_motor(motor, mesh_size=2.0, show_viewer=False)
        print(f"SUCCESS: Mesh generated: {mesh_file}")
    except Exception as e:
        print(f"ERROR: Mesh generation failed: {e}")
        traceback.print_exc()
        return
    
    # Step 2: Solve magnetostatic problem
    print("\n" + "=" * 60)
    print("Step 2: Solving magnetostatic FEA problem...")
    print("=" * 60)
    
    try:
        # Solve and view B-field magnitude
        pos_file = solve_magnetostatic(motor, mesh_file, view_field="norm_b")
        print(f"SUCCESS: Solution completed: {pos_file}")
    except Exception as e:
        print(f"ERROR: FEA solve failed: {e}")
        traceback.print_exc()
        return
    
    # Step 3: Validate solution
    print("\n" + "=" * 60)
    print("Step 3: Validating FEA solution...")
    print("=" * 60)
    
    try:
        import re
        import numpy as np
        norm_b_file = pos_file.replace('.pos', '_norm_b.pos')
        
        # Read the norm_b file and extract values
        with open(norm_b_file, 'r') as f:
            content = f.read()
        
        # Extract all numeric values from the file (excluding nan)
        values = re.findall(r'\{([^}]+)\}', content)
        b_values = []
        nan_count = 0
        
        for val_group in values:
            nums = val_group.split(',')
            for num in nums:
                num = num.strip()
                if num.lower() == 'nan':
                    nan_count += 1
                else:
                    try:
                        b_values.append(float(num))
                    except ValueError:
                        pass
        
        if len(b_values) == 0:
            print(f"ERROR: No valid B-field values found! All {nan_count} values are NaN.")
            print("This indicates the FEA solution failed to converge properly.")
            print("Check boundary conditions and material properties.")
            return
        
        b_array = np.array(b_values)
        b_max = np.max(b_array)
        b_min = np.min(b_array)
        b_mean = np.mean(b_array)
        
        print(f"\nB-field statistics:")
        print(f"  Valid values: {len(b_values)}")
        print(f"  NaN values: {nan_count}")
        print(f"  Max B-field: {b_max:.4f} T")
        print(f"  Min B-field: {b_min:.4f} T")
        print(f"  Mean B-field: {b_mean:.4f} T")
        
        # Check if we have reasonable B-field values (should be around 1T in magnets/airgap)
        if b_max < 0.1:
            print(f"\nWARNING: Maximum B-field is very low ({b_max:.4f} T). Expected ~1T.")
            print("Solution may not be correct.")
            return
        elif b_max > 10.0:
            print(f"\nWARNING: Maximum B-field is very high ({b_max:.4f} T). Expected ~1T.")
            print("Solution may not be correct.")
            return
        else:
            print(f"\nSUCCESS: B-field values are in reasonable range (0.1-10 T)")
            print(f"SUCCESS: Peak B-field of {b_max:.4f} T is physically reasonable for motor magnets")
        
    except Exception as e:
        print(f"ERROR: Solution validation failed: {e}")
        traceback.print_exc()
        return
    
    # Step 3: Validate solution
    print("\n" + "=" * 60)
    print("Step 3: Validating FEA solution...")
    print("=" * 60)
    
    try:
        import re
        norm_b_file = pos_file.replace('.pos', '_norm_b.pos')
        
        # Read the norm_b file and extract values
        with open(norm_b_file, 'r') as f:
            content = f.read()
        
        # Extract all numeric values from the file (excluding nan)
        values = re.findall(r'\{([^}]+)\}', content)
        b_values = []
        nan_count = 0
        
        for val_group in values:
            nums = val_group.split(',')
            for num in nums:
                num = num.strip()
                if num.lower() == 'nan':
                    nan_count += 1
                else:
                    try:
                        b_values.append(float(num))
                    except ValueError:
                        pass
        
        if len(b_values) == 0:
            print(f"ERROR: No valid B-field values found! All {nan_count} values are NaN.")
            print("This indicates the FEA solution failed to converge properly.")
            return
        
        import numpy as np
        b_array = np.array(b_values)
        b_max = np.max(b_array)
        b_min = np.min(b_array)
        b_mean = np.mean(b_array)
        
        print(f"\nB-field statistics:")
        print(f"  Valid values: {len(b_values)}")
        print(f"  NaN values: {nan_count}")
        print(f"  Max B-field: {b_max:.4f} T")
        print(f"  Min B-field: {b_min:.4f} T")
        print(f"  Mean B-field: {b_mean:.4f} T")
        
        # Check if we have reasonable B-field values (should be around 1T in magnets/airgap)
        if b_max < 0.1:
            print(f"\nWARNING: Maximum B-field is very low ({b_max:.4f} T). Expected ~1T.")
            print("Solution may not be correct.")
            return
        elif b_max > 10.0:
            print(f"\nWARNING: Maximum B-field is very high ({b_max:.4f} T). Expected ~1T.")
            print("Solution may not be correct.")
            return
        else:
            print(f"\nSUCCESS: B-field values are in reasonable range (0.1-10 T)")
            print(f"SUCCESS: Peak B-field of {b_max:.4f} T is physically reasonable for motor magnets")
        
    except Exception as e:
        print(f"ERROR: Solution validation failed: {e}")
        traceback.print_exc()
        return
    
    print("\n" + "=" * 60)
    print("Test completed successfully!")
    print("=" * 60)
    print("\nGmsh viewer should have opened showing the B-field magnitude.")
    print("\nIn the gmsh window:")
    print("  - Use mouse to pan/zoom")
    print("  - View > Options > View[0] to adjust color scale")
    print("  - Tools > Visibility to show/hide different fields")
    print("  - You can also load other field files (.pos) to compare:")
    print("    - motor_magnetostatic_a.pos (magnetic vector potential)")
    print("    - motor_magnetostatic_b.pos (magnetic flux density vector)")
    print("    - motor_magnetostatic_h.pos (magnetic field intensity)")
    print("    - motor_magnetostatic_norm_b.pos (B-field magnitude)")


if __name__ == "__main__":
    try:
        test_magnetostatic_solver()
    except Exception as e:
        print(f"\nFATAL ERROR: {e}")
        traceback.print_exc()
        sys.exit(1)
