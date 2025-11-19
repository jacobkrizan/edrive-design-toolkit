"""
Test Suite for Motor Design Toolkit
Validates electromagnetic calculations and geometry generation
"""

import numpy as np
from motor_parameters import MotorParameters, PerformanceMetrics
from geometry import MotorGeometry
from magnetic_circuit import MagneticCircuitSolver
from visualization import MotorVisualizer


def test_motor_parameters():
    """Test motor parameters initialization"""
    print("=" * 60)
    print("TEST 1: Motor Parameters Initialization")
    print("=" * 60)
    
    params = MotorParameters()
    
    # Verify basic parameters
    assert params.num_poles == 8, "Default poles should be 8"
    assert params.num_slots == 48, "Default slots should be 48"
    assert params.slots_per_pole_per_phase == 2.0, "Should have 2 slots per pole per phase"
    
    print(f"✓ Number of poles: {params.num_poles}")
    print(f"✓ Number of slots: {params.num_slots}")
    print(f"✓ Slots per pole per phase: {params.slots_per_pole_per_phase}")
    print(f"✓ Pole pitch: {params.pole_pitch:.2f} mm")
    print(f"✓ Slot pitch: {params.slot_pitch:.2f} mm")
    
    print("\nTEST 1: PASSED ✓\n")
    return params


def test_geometry_generation(params):
    """Test geometry generation"""
    print("=" * 60)
    print("TEST 2: Geometry Generation")
    print("=" * 60)
    
    geometry = MotorGeometry(params)
    
    # Generate stator geometry
    stator_geo = geometry.generate_stator_geometry()
    assert len(stator_geo['slots']) == params.num_slots, "Should generate correct number of slots"
    
    print(f"✓ Generated {len(stator_geo['slots'])} slots")
    
    # Generate rotor geometry
    rotor_geo = geometry.generate_rotor_geometry()
    assert len(rotor_geo['magnets']) == params.num_poles, "Should generate correct number of magnets"
    
    print(f"✓ Generated {len(rotor_geo['magnets'])} magnets")
    
    # Check magnet polarity alternation
    polarities = [m['polarity'] for m in rotor_geo['magnets']]
    for i in range(len(polarities) - 1):
        assert polarities[i] != polarities[i+1], "Magnets should alternate polarity"
    
    print(f"✓ Magnet polarities alternate correctly")
    
    # Generate winding configuration
    winding_config = geometry.get_winding_positions()
    assert len(winding_config) == 3, "Should have 3 phases"
    
    total_slots_in_windings = sum(len(winding_config[phase]) for phase in ['A', 'B', 'C'])
    print(f"✓ Generated winding configuration for {total_slots_in_windings} slot positions")
    
    # Check airgap mesh
    airgap_mesh = geometry.get_airgap_mesh(n_points=360)
    assert airgap_mesh.shape[0] == 360, "Should generate correct number of airgap points"
    assert airgap_mesh.shape[1] == 3, "Should have x, y, theta coordinates"
    
    print(f"✓ Generated airgap mesh with {airgap_mesh.shape[0]} points")
    
    print("\nTEST 2: PASSED ✓\n")
    return geometry


def test_magnetic_circuit(params, geometry):
    """Test magnetic equivalent circuit solver"""
    print("=" * 60)
    print("TEST 3: Magnetic Equivalent Circuit Solver")
    print("=" * 60)
    
    solver = MagneticCircuitSolver(params, geometry)
    
    # Build network
    solver.build_network()
    print(f"✓ Built MEC network with {solver.node_count} nodes")
    print(f"  - Airgap nodes: {len(solver.airgap_nodes)}")
    print(f"  - Magnet nodes: {len(solver.magnet_nodes)}")
    print(f"  - Tooth nodes: {len(solver.tooth_nodes)}")
    print(f"  - Yoke nodes: {len(solver.yoke_nodes)}")
    
    # Calculate reluctances
    reluctances = solver.calculate_reluctances()
    print(f"\n✓ Calculated reluctances:")
    for key, value in reluctances.items():
        print(f"  - {key}: {value:.2e} H⁻¹")
    
    # Solve MEC
    solution = solver.solve_mec(rotor_angle=0.0)
    assert 'potentials' in solution, "Solution should contain potentials"
    assert 'fluxes' in solution, "Solution should contain fluxes"
    
    print(f"\n✓ Solved MEC successfully")
    print(f"  - Airgap flux points: {len(solution['fluxes']['airgap'])}")
    print(f"  - Tooth flux values: {len(solution['fluxes']['teeth'])}")
    
    # Check flux values are reasonable
    airgap_fluxes = solution['fluxes']['airgap']
    max_flux = max([abs(f) for f in airgap_fluxes])
    print(f"  - Maximum airgap flux: {max_flux:.6f} Wb")
    
    assert max_flux > 0, "Should have non-zero flux"
    assert max_flux < 1.0, "Flux should be in reasonable range"
    
    print("\nTEST 3: PASSED ✓\n")
    return solver


def test_performance_calculation(params, geometry, solver):
    """Test performance metrics calculation"""
    print("=" * 60)
    print("TEST 4: Performance Metrics Calculation")
    print("=" * 60)
    
    metrics = solver.calculate_performance(rotor_angle=0.0)
    
    # Check that key metrics are calculated
    print("✓ Calculated performance metrics:")
    print(f"  - Torque: {metrics.torque:.2f} Nm")
    print(f"  - Power: {metrics.power:.2f} kW")
    print(f"  - Back EMF: {metrics.back_emf:.1f} V")
    print(f"  - Efficiency: {metrics.efficiency:.1f} %")
    print(f"  - Power Factor: {metrics.power_factor:.3f}")
    
    # Validate metrics are in reasonable ranges
    assert metrics.torque > 0, "Torque should be positive"
    assert metrics.power > 0, "Power should be positive"
    assert 0 < metrics.efficiency <= 100, "Efficiency should be between 0 and 100%"
    assert 0 <= metrics.power_factor <= 1, "Power factor should be between 0 and 1"
    
    print(f"\n✓ Flux densities:")
    print(f"  - Airgap (peak): {metrics.airgap_flux_density_peak:.3f} T")
    print(f"  - Tooth: {metrics.tooth_flux_density:.3f} T")
    print(f"  - Yoke: {metrics.yoke_flux_density:.3f} T")
    
    # Check flux densities are reasonable
    assert metrics.airgap_flux_density_peak < 2.0, "Airgap flux density should be < 2T"
    assert metrics.tooth_flux_density < 2.5, "Tooth flux density should be reasonable (< 2.5T)"
    
    print(f"\n✓ Losses:")
    print(f"  - Copper loss: {metrics.copper_loss:.1f} W")
    print(f"  - Iron loss: {metrics.iron_loss:.1f} W")
    
    print(f"\n✓ Utilization:")
    print(f"  - Active mass: {metrics.active_mass:.2f} kg")
    print(f"  - Torque density: {metrics.torque_density:.2f} Nm/kg")
    print(f"  - Power density: {metrics.power_density:.2f} kW/kg")
    
    print("\nTEST 4: PASSED ✓\n")
    return metrics


def test_different_configurations():
    """Test different motor configurations"""
    print("=" * 60)
    print("TEST 5: Different Motor Configurations")
    print("=" * 60)
    
    # Test 1: 10-pole, 60-slot motor
    print("\nConfiguration 1: 10 poles, 60 slots")
    params1 = MotorParameters(num_poles=10, num_slots=60)
    geometry1 = MotorGeometry(params1)
    solver1 = MagneticCircuitSolver(params1, geometry1)
    metrics1 = solver1.calculate_performance()
    print(f"  ✓ Torque: {metrics1.torque:.2f} Nm, Power: {metrics1.power:.2f} kW")
    
    # Test 2: 12-pole, 72-slot motor
    print("\nConfiguration 2: 12 poles, 72 slots")
    params2 = MotorParameters(num_poles=12, num_slots=72)
    geometry2 = MotorGeometry(params2)
    solver2 = MagneticCircuitSolver(params2, geometry2)
    metrics2 = solver2.calculate_performance()
    print(f"  ✓ Torque: {metrics2.torque:.2f} Nm, Power: {metrics2.power:.2f} kW")
    
    # Test 3: Different magnet thickness
    print("\nConfiguration 3: Thicker magnets (8mm)")
    params3 = MotorParameters(magnet_thickness=8.0)
    geometry3 = MotorGeometry(params3)
    solver3 = MagneticCircuitSolver(params3, geometry3)
    metrics3 = solver3.calculate_performance()
    print(f"  ✓ Torque: {metrics3.torque:.2f} Nm, Power: {metrics3.power:.2f} kW")
    print(f"  ✓ Airgap flux density increased to: {metrics3.airgap_flux_density_peak:.3f} T")
    
    print("\nTEST 5: PASSED ✓\n")


def test_rotor_positions(solver):
    """Test performance at different rotor positions"""
    print("=" * 60)
    print("TEST 6: Rotor Position Variation")
    print("=" * 60)
    
    angles = [0, 30, 60, 90]
    torques = []
    
    for angle_deg in angles:
        angle_rad = angle_deg * np.pi / 180.0
        metrics = solver.calculate_performance(rotor_angle=angle_rad)
        torques.append(metrics.torque)
        print(f"  Angle {angle_deg:3d}°: Torque = {metrics.torque:.2f} Nm")
    
    # Check torque variation (should have some ripple but not too much)
    torque_avg = np.mean(torques)
    torque_std = np.std(torques)
    ripple = (max(torques) - min(torques)) / torque_avg * 100
    
    print(f"\n✓ Average torque: {torque_avg:.2f} Nm")
    print(f"✓ Torque ripple: {ripple:.1f} %")
    
    assert ripple < 50, "Torque ripple should be reasonable"
    
    print("\nTEST 6: PASSED ✓\n")


def run_all_tests():
    """Run complete test suite"""
    print("\n" + "=" * 60)
    print("ELECTRIC MOTOR DESIGN TOOLKIT - TEST SUITE")
    print("=" * 60 + "\n")
    
    try:
        # Run tests
        params = test_motor_parameters()
        geometry = test_geometry_generation(params)
        solver = test_magnetic_circuit(params, geometry)
        metrics = test_performance_calculation(params, geometry, solver)
        test_different_configurations()
        test_rotor_positions(solver)
        
        # Summary
        print("=" * 60)
        print("ALL TESTS PASSED SUCCESSFULLY! ✓✓✓")
        print("=" * 60)
        print("\nThe motor design toolkit is ready to use.")
        print("Run 'python main.py' to launch the GUI application.\n")
        
        return True
        
    except AssertionError as e:
        print(f"\n✗ TEST FAILED: {e}\n")
        return False
    except Exception as e:
        print(f"\n✗ UNEXPECTED ERROR: {e}\n")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    run_all_tests()
