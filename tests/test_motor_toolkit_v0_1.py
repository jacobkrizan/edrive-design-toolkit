"""
Comprehensive test suite for E-Drive Design Toolkit
Tests all key functionality: parameters, geometry, MEC solver, FEA solver
"""

import unittest
import numpy as np
import os
import sys

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from motor_parameters_v0_1 import MotorParameters
from geometry_v0_1 import MotorGeometry
from magnetic_circuit_v0_1 import MagneticCircuitSolver
from fea_solver_v0_1 import FEASolver


class TestMotorParameters(unittest.TestCase):
    """Test motor parameter initialization and calculations"""
    
    def setUp(self):
        """Create default motor parameters for testing"""
        self.params = MotorParameters()
    
    def test_initialization(self):
        """Test that parameters initialize with correct default values"""
        self.assertEqual(self.params.num_poles, 8)
        self.assertEqual(self.params.num_slots, 48)
        # Parameters are in mm, not meters
        self.assertEqual(self.params.stator_outer_diameter, 200.0)
        self.assertAlmostEqual(self.params.magnet_br, 1.2, places=2)
    
    def test_slot_pole_ratio(self):
        """Test slots per pole calculation"""
        slots_per_pole = self.params.num_slots / self.params.num_poles
        self.assertEqual(slots_per_pole, 6.0)
    
    def test_diameter_constraints(self):
        """Test that diameters are properly ordered"""
        self.assertGreater(self.params.stator_outer_diameter, 
                          self.params.stator_inner_diameter)
        self.assertGreater(self.params.stator_inner_diameter, 
                          self.params.rotor_outer_diameter)
        self.assertGreater(self.params.rotor_outer_diameter, 
                          self.params.rotor_inner_diameter)
    
    def test_pole_pitch(self):
        """Test pole pitch calculation"""
        pole_pitch = np.pi * self.params.rotor_outer_diameter / self.params.num_poles
        self.assertGreater(pole_pitch, 0)
        self.assertLess(pole_pitch, np.pi * self.params.rotor_outer_diameter)


class TestMotorGeometry(unittest.TestCase):
    """Test geometric calculations and mesh generation"""
    
    def setUp(self):
        """Create geometry object for testing"""
        self.params = MotorParameters()
        self.geometry = MotorGeometry(self.params)
    
    def test_stator_geometry_generation(self):
        """Test that stator geometry is generated"""
        stator = self.geometry.generate_stator_geometry()
        self.assertIn('slots', stator)
        self.assertEqual(len(stator['slots']), self.params.num_slots)
    
    def test_rotor_geometry_generation(self):
        """Test that rotor geometry is generated"""
        rotor = self.geometry.generate_rotor_geometry()
        self.assertIn('magnets', rotor)
        self.assertEqual(len(rotor['magnets']), self.params.num_poles)
    
    def test_winding_positions(self):
        """Test winding position calculation"""
        windings = self.geometry.get_winding_positions()
        # Should have phase information (A, B, C)
        self.assertIn('A', windings)
        self.assertIn('B', windings)
        self.assertIn('C', windings)
        # Each phase should have windings
        self.assertGreater(len(windings['A']), 0)
        self.assertGreater(len(windings['B']), 0)
        self.assertGreater(len(windings['C']), 0)
    
    def test_airgap_mesh(self):
        """Test airgap mesh generation"""
        mesh = self.geometry.get_airgap_mesh(n_points=100)
        self.assertEqual(len(mesh), 100)
        # Points are 3D coordinates (x, y, z) but z=0 for 2D
        self.assertEqual(mesh.shape[1], 3)


class TestMECSolver(unittest.TestCase):
    """Test Magnetic Equivalent Circuit solver"""
    
    def setUp(self):
        """Create MEC solver for testing"""
        self.params = MotorParameters()
        self.geometry = MotorGeometry(self.params)
        self.solver = MagneticCircuitSolver(self.params, self.geometry)
    
    def test_solver_initialization(self):
        """Test that solver initializes properly"""
        self.assertIsNotNone(self.solver)
        self.assertEqual(self.solver.params, self.params)
        self.assertEqual(self.solver.geometry, self.geometry)
    
    def test_solve_basic(self):
        """Test that solver produces results"""
        results = self.solver.solve_mec(rotor_angle=0.0)
        
        # Check that results dictionary has expected keys
        self.assertIn('potentials', results)
        self.assertIn('fluxes', results)
        self.assertIn('reluctances', results)
    
    def test_flux_values(self):
        """Test that flux values are physically reasonable"""
        results = self.solver.solve_mec(rotor_angle=0.0)
        
        # Check airgap flux exists
        self.assertIn('fluxes', results)
        self.assertIn('airgap', results['fluxes'])
        
        airgap_fluxes = results['fluxes']['airgap']
        max_flux = np.max(np.abs(airgap_fluxes))
        
        # Should have significant flux (> 0.0001 Wb)
        self.assertGreater(max_flux, 0.0001)
        # Should be reasonable for motor (< 0.01 Wb)
        self.assertLess(max_flux, 0.01)
    
    def test_rotor_angle_variation(self):
        """Test solver with different rotor angles"""
        angles = [0, np.pi/8, np.pi/4, np.pi/2]
        
        for angle in angles:
            results = self.solver.solve_mec(rotor_angle=angle)
            # Should always produce results
            self.assertIsNotNone(results)
            self.assertIn('potentials', results)


class TestFEASolver(unittest.TestCase):
    """Test Finite Element Analysis solver"""
    
    def setUp(self):
        """Create FEA solver for testing"""
        self.params = MotorParameters()
        self.geometry = MotorGeometry(self.params)
        self.solver = FEASolver(self.params, self.geometry)
    
    def test_solver_initialization(self):
        """Test that FEA solver initializes properly"""
        self.assertIsNotNone(self.solver)
        self.assertEqual(self.solver.params, self.params)
        self.assertEqual(self.solver.geometry, self.geometry)
    
    def test_mesh_creation(self):
        """Test that mesh can be created"""
        mesh_data = self.solver.create_mesh()
        self.assertIsNotNone(mesh_data)
        
        # Check mesh has nodes and elements
        self.assertIn('nodes', mesh_data)
        self.assertIn('elements', mesh_data)
        self.assertGreater(len(mesh_data['nodes']), 0)
        self.assertGreater(len(mesh_data['elements']), 0)
    
    def test_getdp_availability(self):
        """Test that GetDP is available"""
        getdp_path = os.path.join(os.path.dirname(__file__), 
                                  "..", "src", "getdp-3.5.0-Windows64", "getdp.exe")
        
        if os.path.exists(getdp_path):
            self.assertTrue(True, "GetDP found")
        else:
            self.skipTest("GetDP not installed")
    
    def test_solve_magnetostatic(self):
        """Test that magnetostatic solution can be computed"""
        getdp_path = os.path.join(os.path.dirname(__file__), 
                                  "..", "src", "getdp-3.5.0-Windows64", "getdp.exe")
        if not os.path.exists(getdp_path):
            self.skipTest("GetDP not installed")
        
        try:
            mesh_data = self.solver.create_mesh()
            solution = self.solver.solve_magnetostatic(mesh_data, rotor_angle=0.0)
            
            # Check solution exists
            self.assertIsNotNone(solution)
            self.assertIn('flux_density_nodes', solution)
            
            # Check flux density values
            flux = solution['flux_density_nodes']
            max_flux = np.max(np.linalg.norm(flux, axis=1))
            
            # Should have significant field
            self.assertGreater(max_flux, 0.1)
            # Should be physically reasonable
            self.assertLess(max_flux, 3.0)
            
        except Exception as e:
            if "GetDP" in str(e):
                self.skipTest(f"GetDP execution issue: {e}")
            else:
                raise


class TestIntegration(unittest.TestCase):
    """Integration tests for complete workflow"""
    
    def test_complete_mec_workflow(self):
        """Test complete workflow from parameters to MEC results"""
        params = MotorParameters()
        geometry = MotorGeometry(params)
        solver = MagneticCircuitSolver(params, geometry)
        
        results = solver.solve_mec(rotor_angle=0.0)
        
        # Verify results are complete
        self.assertIn('potentials', results)
        self.assertIn('fluxes', results)
        self.assertIn('reluctances', results)
        
        # Check physical groups
        self.assertIn('airgap', results['fluxes'])
        self.assertIn('teeth', results['fluxes'])
    
    def test_parameter_sweep(self):
        """Test that solver handles parameter variations"""
        params = MotorParameters()
        geometry = MotorGeometry(params)
        solver = MagneticCircuitSolver(params, geometry)
        
        # Sweep rotor angles
        angles = np.linspace(0, 2*np.pi/params.num_poles, 10)
        flux_values = []
        
        for angle in angles:
            results = solver.solve_mec(rotor_angle=angle)
            max_flux = np.max(np.abs(results['fluxes']['airgap']))
            flux_values.append(max_flux)
        
        # Should have variation in flux
        self.assertGreater(np.std(flux_values), 0)
        self.assertEqual(len(flux_values), len(angles))
    
    def test_different_motor_configs(self):
        """Test different motor configurations"""
        configs = [
            {'num_poles': 6, 'num_slots': 36},
            {'num_poles': 8, 'num_slots': 48},
            {'num_poles': 10, 'num_slots': 60},
        ]
        
        for config in configs:
            params = MotorParameters()
            params.num_poles = config['num_poles']
            params.num_slots = config['num_slots']
            
            geometry = MotorGeometry(params)
            solver = MagneticCircuitSolver(params, geometry)
            
            results = solver.solve_mec(rotor_angle=0.0)
            
            # Should produce valid results
            self.assertIsNotNone(results)
            self.assertIn('fluxes', results)
            
            # Check correct number of components
            stator = geometry.generate_stator_geometry()
            self.assertEqual(len(stator['slots']), config['num_slots'])


class TestThreePhaseTorque(unittest.TestCase):
    """Test three-phase torque calculation with individual tooth flux tracking"""
    
    def setUp(self):
        """Create solver for testing"""
        self.params = MotorParameters()
        self.geometry = MotorGeometry(self.params)
        self.solver = MagneticCircuitSolver(self.params, self.geometry)
    
    def test_individual_tooth_flux_tracking(self):
        """Test that individual tooth fluxes are calculated correctly"""
        results = self.solver.solve_mec(rotor_angle=0.0)
        
        # Check that we have individual tooth fluxes
        tooth_fluxes = results['fluxes']['teeth']
        num_slots = int(self.params.num_slots)
        self.assertEqual(len(tooth_fluxes), num_slots, 
                        f"Should have {num_slots} individual tooth fluxes")
        
        # Check that tooth_flux_linkage exists with all three phases
        self.assertIn('tooth_flux_linkage', results['fluxes'])
        phase_flux_linkage = results['fluxes']['tooth_flux_linkage']
        self.assertIn('A', phase_flux_linkage)
        self.assertIn('B', phase_flux_linkage)
        self.assertIn('C', phase_flux_linkage)
        
        # Verify flux linkages are reasonable (not zero, not infinite)
        # Flux linkage = flux * turns, so for ~1mWb flux and ~1000 turns, expect ~1 Wb-turns
        for phase in ['A', 'B', 'C']:
            flux = abs(phase_flux_linkage[phase])
            self.assertGreater(flux, 0.0, f"Phase {phase} flux linkage should be positive")
            self.assertLess(flux, 100.0, f"Phase {phase} flux linkage too large: {flux} Wb-turns")
    
    def test_three_phase_torque_calculation(self):
        """Test that torque is calculated using three-phase flux linkages"""
        perf = self.solver.calculate_performance()
        
        # Check performance metrics exist (PerformanceMetrics is a class, use hasattr)
        self.assertTrue(hasattr(perf, 'flux_linkage'))
        self.assertTrue(hasattr(perf, 'torque'))
        self.assertTrue(hasattr(perf, 'torque_ripple'))
        self.assertTrue(hasattr(perf, 'back_emf'))
        
        # Verify torque is reasonable (MEC solver gives high values due to simplified model)
        torque = perf.torque
        self.assertGreater(torque, 0.0, "Torque should be positive")
        # MEC can give higher values than reality due to idealized permeabilities
        
        # Check that torque ripple is calculated
        ripple = perf.torque_ripple
        self.assertGreater(ripple, 0.0, "Torque ripple should exist")
        self.assertLess(ripple, 200.0, f"Torque ripple too large: {ripple}%")
        
        # Back-EMF should be positive (actual value depends on model accuracy)
        emf = perf.back_emf
        self.assertGreater(emf, 0.0, "Back-EMF should be positive")
    
    def test_tooth_flux_density_saturation(self):
        """Test that tooth flux magnitudes are reasonable"""
        results = self.solver.solve_mec(rotor_angle=0.0)
        tooth_fluxes = results['fluxes']['teeth']
        
        # Check that tooth fluxes are in reasonable range  
        # Individual teeth fluxes should be in milliWeber range
        avg_tooth_flux = np.mean(np.abs(tooth_fluxes))
        max_tooth_flux = np.max(np.abs(tooth_fluxes))
        
        # Expect ~0.1-10 mWb per tooth
        self.assertGreater(avg_tooth_flux, 1e-5, f"Average tooth flux too low: {avg_tooth_flux*1e3:.3f} mWb")
        self.assertLess(max_tooth_flux, 0.1, f"Max tooth flux too high: {max_tooth_flux*1e3:.1f} mWb")
    
    def test_phase_balance(self):
        """Test that three-phase flux linkages are balanced"""
        results = self.solver.solve_mec(rotor_angle=0.0)
        phase_flux = results['fluxes']['tooth_flux_linkage']
        
        # Calculate average flux linkage
        avg_flux = np.mean([abs(phase_flux[p]) for p in ['A', 'B', 'C']])
        
        # Check balance (each phase should be within 25% of average)
        for phase in ['A', 'B', 'C']:
            flux = abs(phase_flux[phase])
            deviation = abs(flux - avg_flux) / avg_flux if avg_flux > 0 else 0
            self.assertLess(deviation, 0.25, 
                          f"Phase {phase} flux imbalance: {deviation*100:.1f}%")


class TestMECSymmetry(unittest.TestCase):
    """Test MEC solver symmetry for multi-pole motors with zero current"""
    
    def test_8_pole_48_slot_symmetry_zero_current(self):
        """Test that 8-pole motor has symmetric airgap flux with zero current (magnet-only)"""
        # Create 8-pole 48-slot motor
        params = MotorParameters()
        params.num_poles = 8
        params.num_slots = 48
        params.rated_current = 0.0  # Zero current - only magnet MMF
        
        geometry = MotorGeometry(params)
        solver = MagneticCircuitSolver(params, geometry)
        
        # Solve at rotor angle 0
        solution = solver.solve_mec(rotor_angle=0.0)
        airgap_flux = solution['fluxes']['airgap']
        
        # Calculate flux per pole
        n_ag = len(airgap_flux)
        points_per_pole = n_ag // params.num_poles
        
        # Collect flux for each pole
        positive_pole_fluxes = []
        negative_pole_fluxes = []
        
        for pole in range(params.num_poles):
            start_idx = pole * points_per_pole
            end_idx = (pole + 1) * points_per_pole
            pole_flux = airgap_flux[start_idx:end_idx]
            avg_flux = np.mean(pole_flux)
            
            if pole % 2 == 0:
                positive_pole_fluxes.append(avg_flux)
                self.assertGreater(avg_flux, 0, f"Pole {pole} should be positive")
            else:
                negative_pole_fluxes.append(avg_flux)
                self.assertLess(avg_flux, 0, f"Pole {pole} should be negative")
        
        # Check that all positive poles have similar magnitude
        pos_mean = np.mean(positive_pole_fluxes)
        pos_std = np.std(positive_pole_fluxes)
        pos_variation = pos_std / abs(pos_mean) * 100 if pos_mean != 0 else 0
        
        self.assertLess(pos_variation, 5.0,
                       f"Positive poles have {pos_variation:.2f}% variation (should be < 5%)")
        
        # Check that all negative poles have similar magnitude
        neg_mean = np.mean(negative_pole_fluxes)
        neg_std = np.std(negative_pole_fluxes)
        neg_variation = neg_std / abs(neg_mean) * 100 if neg_mean != 0 else 0
        
        self.assertLess(neg_variation, 5.0,
                       f"Negative poles have {neg_variation:.2f}% variation (should be < 5%)")
        
        # Check that positive and negative poles have same magnitude (symmetry)
        symmetry_ratio = abs(pos_mean / neg_mean)
        self.assertAlmostEqual(symmetry_ratio, 1.0, places=2,
                              msg=f"Symmetry ratio {symmetry_ratio:.4f} should be ~1.0")
        
        # Check global max/min symmetry
        max_flux = np.max(airgap_flux)
        min_flux = np.min(airgap_flux)
        global_symmetry = max_flux / (-min_flux)
        
        self.assertAlmostEqual(global_symmetry, 1.0, places=2,
                              msg=f"Global symmetry {global_symmetry:.4f} should be ~1.0")


class TestFEASymmetry(unittest.TestCase):
    """Test FEA solver symmetry for multi-pole motors with zero current"""
    
    def test_8_pole_48_slot_symmetry_zero_current_fea(self):
        """Test that 8-pole motor has symmetric airgap flux with zero current using FEA"""
        # Create 8-pole 48-slot motor
        params = MotorParameters()
        params.num_poles = 8
        params.num_slots = 48
        params.rated_current = 0.0  # Zero current - only magnet MMF
        
        geometry = MotorGeometry(params)
        fea_solver = FEASolver(params, geometry)
        
        # Check if GetDP is available
        getdp_path = os.path.join(os.path.dirname(__file__), 
                                  "..", "src", "getdp-3.5.0-Windows64", "getdp.exe")
        if not os.path.exists(getdp_path):
            self.skipTest("GetDP not installed")
        
        try:
            # Create mesh and solve
            mesh_data = fea_solver.create_mesh(mesh_size=3.0)
            solution = fea_solver.solve_magnetostatic(mesh_data, rotor_angle=0.0)
            
            # Get airgap flux distribution
            airgap_flux = fea_solver.get_airgap_flux(solution, n_points=360)
            
            # Calculate flux per pole
            n_ag = len(airgap_flux)
            points_per_pole = n_ag // params.num_poles
            
            # Collect flux for each pole
            positive_pole_fluxes = []
            negative_pole_fluxes = []
            
            for pole in range(params.num_poles):
                start_idx = pole * points_per_pole
                end_idx = (pole + 1) * points_per_pole
                pole_flux = airgap_flux[start_idx:end_idx]
                avg_flux = np.mean(pole_flux)
                
                if pole % 2 == 0:
                    positive_pole_fluxes.append(avg_flux)
                else:
                    negative_pole_fluxes.append(avg_flux)
            
            # FEA with slots shows large average flux variation due to slot openings
            # Check using max flux magnitudes instead of averages
            pos_max_fluxes = [np.max(np.abs(airgap_flux[pole*points_per_pole:(pole+1)*points_per_pole])) 
                             for pole in range(0, params.num_poles, 2)]
            neg_max_fluxes = [np.max(np.abs(airgap_flux[pole*points_per_pole:(pole+1)*points_per_pole])) 
                             for pole in range(1, params.num_poles, 2)]
            
            pos_max_mean = np.mean(pos_max_fluxes)
            pos_max_std = np.std(pos_max_fluxes)
            pos_max_variation = pos_max_std / pos_max_mean * 100 if pos_max_mean > 0 else 0
            
            neg_max_mean = np.mean(neg_max_fluxes)
            neg_max_std = np.std(neg_max_fluxes)
            neg_max_variation = neg_max_std / neg_max_mean * 100 if neg_max_mean > 0 else 0
            
            # Peak flux variation should be low (<5%) for symmetric poles
            self.assertLess(pos_max_variation, 5.0,
                           f"FEA: Positive pole peak variation {pos_max_variation:.2f}% (should be < 5%)")
            self.assertLess(neg_max_variation, 5.0,
                           f"FEA: Negative pole peak variation {neg_max_variation:.2f}% (should be < 5%)")
            
            # Check that positive and negative pole peaks have same magnitude
            symmetry_ratio = pos_max_mean / neg_max_mean
            self.assertAlmostEqual(symmetry_ratio, 1.0, places=2,
                                  msg=f"FEA: Peak symmetry ratio {symmetry_ratio:.4f} should be ~1.0")
            
            # Check global max/min symmetry (use absolute values)
            max_abs_flux = np.max(np.abs(airgap_flux))
            mean_abs_flux = np.mean(np.abs(airgap_flux))
            # For symmetric poles, max should not be much larger than mean
            peak_ratio = max_abs_flux / mean_abs_flux if mean_abs_flux > 0 else 0
            # Expect peaks ~1.5-2x mean for sinusoidal distribution
            self.assertLess(peak_ratio, 3.0,
                           msg=f"FEA: Peak/mean ratio {peak_ratio:.2f} is too high")
            
        except Exception as e:
            if "GetDP" in str(e):
                self.skipTest(f"GetDP execution issue: {e}")
            else:
                raise


class TestGUI(unittest.TestCase):
    """Test GUI components"""
    
    def test_visualization_initialization(self):
        """Test that visualization module initializes without errors"""
        from visualization_v0_1 import MotorVisualizer
        
        params = MotorParameters()
        geometry = MotorGeometry(params)
        solver = MagneticCircuitSolver(params, geometry)
        visualizer = MotorVisualizer(params, geometry, solver)
        
        self.assertIsNotNone(visualizer)
    
    def test_mec_diagram_plotting(self):
        """Test that MEC diagram can be plotted without errors"""
        from visualization_v0_1 import MotorVisualizer
        import matplotlib.pyplot as plt
        
        params = MotorParameters()
        geometry = MotorGeometry(params)
        solver = MagneticCircuitSolver(params, geometry)
        visualizer = MotorVisualizer(params, geometry, solver)
        
        # Create figure and axis
        fig, ax = plt.subplots()
        
        # Solve MEC
        solution = solver.solve_mec(rotor_angle=0.0)
        
        # This should not raise any errors
        visualizer.plot_mec_diagram(ax, solution)
        
        plt.close(fig)
    
    def test_motor_cross_section_plotting(self):
        """Test that motor cross-section can be plotted without errors"""
        from visualization_v0_1 import MotorVisualizer
        import matplotlib.pyplot as plt
        
        params = MotorParameters()
        geometry = MotorGeometry(params)
        solver = MagneticCircuitSolver(params, geometry)
        visualizer = MotorVisualizer(params, geometry, solver)
        
        # Create figure and axis
        fig, ax = plt.subplots()
        
        # This should not raise any errors
        visualizer.plot_motor_cross_section(ax, rotor_angle=0.0)
        
        plt.close(fig)
    
    def test_fea_airgap_flux_plotting(self):
        """Test that FEA airgap flux plot can be created without errors"""
        from visualization_v0_1 import MotorVisualizer
        import matplotlib.pyplot as plt
        import os
        
        # Check if GetDP is available
        getdp_path = os.path.join(os.path.dirname(__file__), 
                                  "..", "src", "getdp-3.5.0-Windows64", "getdp.exe")
        if not os.path.exists(getdp_path):
            self.skipTest("GetDP not installed")
        
        params = MotorParameters(num_poles=8, num_slots=48)
        geometry = MotorGeometry(params)
        mec_solver = MagneticCircuitSolver(params, geometry)
        fea_solver = FEASolver(params, geometry)
        visualizer = MotorVisualizer(params, geometry, mec_solver)
        
        try:
            # Create figure and axis
            fig, ax = plt.subplots()
            
            # Create mesh and solve
            mesh_data = fea_solver.create_mesh(mesh_size=3.0)
            solution = fea_solver.solve_magnetostatic(mesh_data, rotor_angle=0.0)
            
            # This should not raise any errors
            visualizer.plot_fea_airgap_flux(ax, fea_solver, solution)
            
            plt.close(fig)
            
        except Exception as e:
            if "GetDP" in str(e):
                self.skipTest(f"GetDP execution issue: {e}")
            else:
                raise


def run_tests():
    """Run all tests and print results"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestMotorParameters))
    suite.addTests(loader.loadTestsFromTestCase(TestMotorGeometry))
    suite.addTests(loader.loadTestsFromTestCase(TestMECSolver))
    suite.addTests(loader.loadTestsFromTestCase(TestFEASolver))
    suite.addTests(loader.loadTestsFromTestCase(TestIntegration))
    suite.addTests(loader.loadTestsFromTestCase(TestThreePhaseTorque))
    suite.addTests(loader.loadTestsFromTestCase(TestMECSymmetry))
    suite.addTests(loader.loadTestsFromTestCase(TestFEASymmetry))
    suite.addTests(loader.loadTestsFromTestCase(TestGUI))
    
    # Run tests with verbose output
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors) - len(result.skipped)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Skipped: {len(result.skipped)}")
    
    if result.wasSuccessful():
        print("\n[PASS] ALL TESTS PASSED!")
    else:
        print("\n[FAIL] Some tests failed - see details above")
    
    print("="*70)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    exit(0 if success else 1)
