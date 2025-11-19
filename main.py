"""
Electric Motor Design Toolkit - Main Application
Professional UI for permanent magnet motor design and analysis
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from typing import Dict
import numpy as np

from motor_parameters import MotorParameters, PerformanceMetrics
from geometry import MotorGeometry
from magnetic_circuit import MagneticCircuitSolver
from fea_solver import FEASolver
from visualization import MotorVisualizer


class MotorDesignApp:
    """Main application class for motor design toolkit"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("E-Drive Design Toolkit - Permanent Magnet Motor Designer")
        self.root.geometry("1920x1080")
        
        # Initialize motor parameters
        self.params = MotorParameters()
        self.geometry = None
        self.solver = None
        self.fea_solver = None
        self.visualizer = None
        self.metrics = None
        self.fea_solution = None
        self.fea_performance = None
        
        # Solver selection
        self.solver_mode = tk.StringVar(value="Both")  # MEC, FEA, or Both
        
        # Rotor position for animation
        self.rotor_angle = 0.0
        
        # Font scaling
        self.base_width = 1920
        self.base_height = 1080
        self.current_font_scale = 1.0
        self.label_widgets = []  # Store labels for font scaling
        self.title_labels = []  # Store title labels
        self.entry_widgets = []  # Store entry widgets
        self.frame_widgets = []  # Store labelframes for scaling
        self.button_widgets = []  # Store buttons for scaling
        self.combobox_widgets = []  # Store comboboxes for scaling
        
        # Create UI
        self.create_ui()
        
        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)
        
        # Initial calculation (after UI is created)
        self.update_design()
    
    def create_ui(self):
        """Create the user interface"""
        # Main container
        main_container = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left panel - Controls (50% width)
        left_panel = ttk.Frame(main_container)
        main_container.add(left_panel, weight=1)
        
        # Right panel - Visualization (50% width)
        right_panel = ttk.Frame(main_container)
        main_container.add(right_panel, weight=1)
        
        # Create control panel
        self.create_control_panel(left_panel)
        
        # Create visualization panel
        self.create_visualization_panel(right_panel)
    
    def create_control_panel(self, parent):
        """Create parameter controls"""
        # Title
        title = ttk.Label(parent, text="Motor Parameters", font=('Arial', 14, 'bold'))
        title.pack(pady=10, fill=tk.X)
        self.title_labels.append(title)  # Store for font scaling
        
        # Create two-column layout for parameters and results
        columns_frame = ttk.Frame(parent)
        columns_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left column - scrollable parameters (40% of left panel)
        left_column = ttk.Frame(columns_frame, width=380)
        left_column.pack(side=tk.LEFT, fill=tk.BOTH, expand=False)
        
        # Right column - performance metrics (60% of left panel)
        right_column = ttk.Frame(columns_frame)
        right_column.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        # Scrollable frame for parameters
        canvas = tk.Canvas(left_column, width=360)
        scrollbar = ttk.Scrollbar(left_column, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Parameter storage
        self.param_vars = {}
        
        # Basic Configuration
        self.create_param_section(scrollable_frame, "Basic Configuration", [
            ("Number of Poles", "num_poles", 4, 20, 2, True, 2),  # Step=2 for pairs
            ("Number of Slots", "num_slots", 12, 96, 6, True, None),  # Dynamic step based on poles
        ])
        
        # Geometric Parameters
        self.create_param_section(scrollable_frame, "Geometry (mm)", [
            ("Stator Outer Dia.", "stator_outer_diameter", 100, 400, 200),
            ("Stator Inner Dia.", "stator_inner_diameter", 80, 300, 120),
            ("Rotor Outer Dia.", "rotor_outer_diameter", 70, 290, 118),
            ("Rotor Inner Dia.", "rotor_inner_diameter", 20, 150, 40),
            ("Airgap Length", "airgap_length", 0.5, 5.0, 1.0),
            ("Stack Length", "stack_length", 50, 300, 100),
            ("Magnet Thickness", "magnet_thickness", 2, 15, 5),
            ("Slot Depth", "slot_depth", 10, 50, 30),
        ])
        
        # Electrical Parameters
        self.create_param_section(scrollable_frame, "Electrical", [
            ("Rated Current (A)", "rated_current", 10, 500, 100),
            ("Rated Speed (rpm)", "rated_speed", 500, 10000, 3000),
            ("Turns per Coil", "turns_per_coil", 5, 100, 20),
        ])
        
        # Material Properties
        self.create_param_section(scrollable_frame, "Materials", [
            ("Magnet Br (T)", "magnet_br", 0.8, 1.5, 1.2),
            ("Steel µr", "steel_mu_r", 500, 5000, 2000),
            ("Magnet Arc Ratio", "magnet_arc_ratio", 0.5, 1.0, 0.85),
        ])
        
        # Solver selection
        solver_frame = ttk.LabelFrame(scrollable_frame, text="Solver Options", padding=10)
        solver_frame.pack(pady=10, padx=10, fill=tk.X)
        self.frame_widgets.append(solver_frame)
        
        solver_label = ttk.Label(solver_frame, text="Analysis Method:")
        solver_label.grid(row=0, column=0, sticky=tk.W, pady=2)
        self.label_widgets.append(solver_label)
        
        solver_combo = ttk.Combobox(solver_frame, textvariable=self.solver_mode,
                                    values=["MEC", "FEA", "Both"], state="readonly", width=18)
        solver_combo.grid(row=0, column=1, sticky=tk.W, pady=2, padx=5)
        self.combobox_widgets.append(solver_combo)
        
        # Update button
        update_btn = ttk.Button(scrollable_frame, text="Update Design", 
                               command=self.update_design, style='Accent.TButton')
        update_btn.pack(pady=15, padx=10, fill=tk.X)
        self.button_widgets.append(update_btn)
        
        # Animation controls
        anim_frame = ttk.LabelFrame(scrollable_frame, text="Animation", padding=10)
        anim_frame.pack(pady=10, padx=10, fill=tk.X)
        self.frame_widgets.append(anim_frame)  # Store for font scaling
        
        self.angle_var = tk.DoubleVar(value=0)
        angle_slider = ttk.Scale(anim_frame, from_=0, to=360, orient=tk.HORIZONTAL,
                                variable=self.angle_var, command=self.update_rotor_angle)
        angle_slider.pack(fill=tk.X, pady=5)
        
        angle_label = ttk.Label(anim_frame, text="Rotor Angle: 0°")
        angle_label.pack()
        self.angle_label = angle_label
        self.label_widgets.append(angle_label)  # Store for font scaling
        
        # Pack scrollbar and canvas
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Results display in right column
        results_frame = ttk.LabelFrame(right_column, text="Performance Metrics", padding=5)
        results_frame.pack(fill=tk.BOTH, expand=True)
        self.frame_widgets.append(results_frame)  # Store for font scaling
        
        self.results_text = scrolledtext.ScrolledText(results_frame, width=35,
                                                      font=('Courier', 9))
        self.results_text.pack(fill=tk.BOTH, expand=True)
    
    def create_param_section(self, parent, title, params):
        """Create a section of parameters"""
        frame = ttk.LabelFrame(parent, text=title, padding=10)
        frame.pack(pady=5, padx=10, fill=tk.X)
        self.frame_widgets.append(frame)  # Store for font scaling
        
        for param_def in params:
            if len(param_def) == 5:
                label, key, min_val, max_val, default = param_def
                is_int = False
                step = None
            elif len(param_def) == 6:
                label, key, min_val, max_val, default, is_int = param_def
                step = None
            else:
                label, key, min_val, max_val, default, is_int, step = param_def
            
            param_frame = ttk.Frame(frame)
            param_frame.pack(fill=tk.X, pady=3)
            
            label_widget = ttk.Label(param_frame, text=label, width=20)
            label_widget.pack(side=tk.LEFT)
            self.label_widgets.append(label_widget)  # Store for font scaling
            
            var = tk.DoubleVar(value=default)
            self.param_vars[key] = var
            
            # Entry
            entry = ttk.Entry(param_frame, textvariable=var, width=10)
            entry.pack(side=tk.RIGHT, padx=5)
            self.entry_widgets.append(entry)  # Store for font scaling
            
            # Slider with rounding for integer params
            if max_val - min_val <= 100:
                if is_int:
                    # Integer slider - round values with step
                    def make_int_command(v, k, s):
                        def cmd(val):
                            rounded = round(float(val))
                            # Apply step constraint
                            if s is not None and s > 1:
                                rounded = round(rounded / s) * s
                            # Special handling for slots - must be multiple of 3*poles
                            if k == 'num_slots' and 'num_poles' in self.param_vars:
                                poles = int(self.param_vars['num_poles'].get())
                                slots_per_pole = max(3, rounded // poles)
                                # Round to nearest multiple of 3
                                slots_per_pole = round(slots_per_pole / 3) * 3
                                rounded = poles * slots_per_pole
                            v.set(rounded)
                            # Auto-update slots when poles change
                            if k == 'num_poles' and 'num_slots' in self.param_vars:
                                self.update_slots_from_poles()
                        return cmd
                    slider = ttk.Scale(param_frame, from_=min_val, to=max_val,
                                     orient=tk.HORIZONTAL, variable=var,
                                     command=make_int_command(var, key, step))
                else:
                    slider = ttk.Scale(param_frame, from_=min_val, to=max_val,
                                     orient=tk.HORIZONTAL, variable=var)
                slider.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)
    
    def create_visualization_panel(self, parent):
        """Create visualization panel with tabbed plots"""
        # Create notebook for tabbed interface
        self.plot_notebook = ttk.Notebook(parent)
        self.plot_notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create tabs for each visualization
        self.plot_tabs = {}
        self.plot_figures = {}
        self.plot_canvases = {}
        
        tab_names = [
            "Motor Cross-Section",
            "Flux Density",
            "Airgap Flux",
            "MEC Diagram",
            "FEA Mesh",
            "FEA Results"
        ]
        
        for tab_name in tab_names:
            # Create tab frame
            tab_frame = ttk.Frame(self.plot_notebook)
            self.plot_notebook.add(tab_frame, text=tab_name)
            self.plot_tabs[tab_name] = tab_frame
            
            # Create figure for this tab
            fig = Figure(figsize=(10, 8), dpi=100)
            self.plot_figures[tab_name] = fig
            
            # Create canvas
            canvas = FigureCanvasTkAgg(fig, master=tab_frame)
            canvas.draw()
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            self.plot_canvases[tab_name] = canvas
            
            # Add toolbar
            from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
            toolbar = NavigationToolbar2Tk(canvas, tab_frame)
            toolbar.update()
    
    def update_design(self):
        """Update motor design based on current parameters"""
        # Update parameters from UI
        for key, var in self.param_vars.items():
            value = var.get()
            # Convert to int for pole and slot counts
            if key in ['num_poles', 'num_slots', 'turns_per_coil', 'parallel_paths', 'winding_layers', 'coil_pitch']:
                value = int(round(value))
                
                # Enforce constraints
                if key == 'num_poles':
                    # Poles must be even (pairs)
                    value = max(4, round(value / 2) * 2)
                    var.set(value)
                elif key == 'num_slots':
                    # Slots must be multiple of 3 * poles for 3-phase winding
                    poles = int(self.param_vars['num_poles'].get())
                    slots_per_pole = max(3, value // poles)
                    slots_per_pole = round(slots_per_pole / 3) * 3
                    value = poles * slots_per_pole
                    var.set(value)
                    
            setattr(self.params, key, value)
        
        # Recalculate derived parameters
        self.params.__post_init__()
        
        # Recreate geometry and solvers
        self.geometry = MotorGeometry(self.params)
        self.solver = MagneticCircuitSolver(self.params, self.geometry)
        self.fea_solver = FEASolver(self.params, self.geometry)
        self.visualizer = MotorVisualizer(self.params, self.geometry, self.solver)
        
        # Run selected solver(s)
        solver_mode = self.solver_mode.get()
        
        if solver_mode in ["MEC", "Both"]:
            self.metrics = self.solver.calculate_performance(self.rotor_angle)
        
        if solver_mode in ["FEA", "Both"]:
            try:
                self.fea_solution, self.fea_performance = self.fea_solver.run_analysis(
                    rotor_angle=self.rotor_angle * 180 / np.pi,
                    mesh_size=3.0
                )
            except Exception as e:
                print(f"FEA error: {e}")
                self.fea_solution = None
                self.fea_performance = None
        
        # Update each visualization tab
        self.update_all_plots()
        
        # Update results text
        self.results_text.delete(1.0, tk.END)
        
        if solver_mode == "MEC" and self.metrics:
            self.results_text.insert(1.0, self.metrics.format_output())
        elif solver_mode == "FEA" and self.fea_performance:
            self.results_text.insert(1.0, self._format_fea_output(self.fea_performance))
        elif solver_mode == "Both":
            output = "=== MEC SOLVER ===\n"
            if self.metrics:
                output += self.metrics.format_output()
            if self.fea_performance:
                output += "\n\n=== FEA SOLVER ===\n"
                output += self._format_fea_output(self.fea_performance)
            self.results_text.insert(1.0, output)
    
    def update_all_plots(self):
        """Update all visualization tabs"""
        if not self.visualizer:
            return
        
        # Get MEC solution
        mec_solution = self.solver.solve_mec(self.rotor_angle) if self.solver else None
        
        # Tab 1: Motor Cross-Section
        fig1 = self.plot_figures["Motor Cross-Section"]
        fig1.clear()
        ax1 = fig1.add_subplot(111)
        self.visualizer.plot_motor_cross_section(ax1, self.rotor_angle)
        self.plot_canvases["Motor Cross-Section"].draw()
        
        # Tab 2: Flux Density Distribution
        fig2 = self.plot_figures["Flux Density"]
        fig2.clear()
        ax2 = fig2.add_subplot(111)
        if mec_solution:
            self.visualizer.plot_flux_density(ax2, mec_solution, self.rotor_angle)
        self.plot_canvases["Flux Density"].draw()
        
        # Tab 3: Airgap Flux
        fig3 = self.plot_figures["Airgap Flux"]
        fig3.clear()
        ax3 = fig3.add_subplot(111)
        if mec_solution:
            self.visualizer.plot_airgap_flux(ax3, mec_solution)
        self.plot_canvases["Airgap Flux"].draw()
        
        # Tab 4: MEC Diagram
        fig4 = self.plot_figures["MEC Diagram"]
        fig4.clear()
        ax4 = fig4.add_subplot(111)
        if mec_solution:
            self.visualizer.plot_mec_diagram(ax4, mec_solution)
        self.plot_canvases["MEC Diagram"].draw()
        
        # Tab 5: FEA Mesh with Geometry Overlay
        fig5 = self.plot_figures["FEA Mesh"]
        fig5.clear()
        if self.fea_solution is not None and 'mesh' in self.fea_solution:
            ax5 = fig5.add_subplot(111)
            self.visualizer.plot_fea_mesh(ax5, self.fea_solution['mesh'], self.rotor_angle)
        else:
            ax5 = fig5.add_subplot(111)
            ax5.text(0.5, 0.5, 'No FEA Mesh\n\nSelect "FEA" or "Both" solver mode',
                    ha='center', va='center', fontsize=14, color='gray')
            ax5.axis('off')
        self.plot_canvases["FEA Mesh"].draw()
        
        # Tab 6: FEA Results (if available)
        fig6 = self.plot_figures["FEA Results"]
        fig6.clear()
        if self.fea_solution is not None:
            ax6 = fig6.add_subplot(111)
            self.visualizer.plot_fea_results(ax6, self.fea_solution)
        else:
            ax6 = fig6.add_subplot(111)
            ax6.text(0.5, 0.5, 'No FEA Results\n\nSelect "FEA" or "Both" solver mode',
                    ha='center', va='center', fontsize=14, color='gray')
            ax6.axis('off')
        self.plot_canvases["FEA Results"].draw()
    
    def _format_fea_output(self, fea_perf: Dict) -> str:
        """Format FEA results"""
        return f"""
Torque:              {fea_perf['torque']:.2f} Nm
Power:               {fea_perf['power']:.2f} kW
Back EMF:            {fea_perf['back_emf']:.1f} V

Flux Densities:
  Airgap:            {fea_perf['airgap_flux_density']:.3f} T
  Peak:              {fea_perf['peak_flux_density']:.3f} T

Mesh:
  Nodes:             {fea_perf['mesh_nodes']}
  Elements:          {fea_perf['mesh_elements']}
"""
    
    def update_rotor_angle(self, value):
        """Update rotor position"""
        self.rotor_angle = float(value) * np.pi / 180.0
        self.angle_label.config(text=f"Rotor Angle: {float(value):.1f}°")
        
        if self.visualizer:
            # Recalculate MEC
            solver_mode = self.solver_mode.get()
            
            if solver_mode in ["MEC", "Both"]:
                self.metrics = self.solver.calculate_performance(self.rotor_angle)
            
            # Update visualizations
            self.update_all_plots()
            
            # Update results
            self.results_text.delete(1.0, tk.END)
            if solver_mode in ["MEC", "Both"] and self.metrics:
                self.results_text.insert(1.0, self.metrics.format_output())
    
    def update_slots_from_poles(self):
        """Auto-update slots to nearest valid value when poles change"""
        if 'num_poles' in self.param_vars and 'num_slots' in self.param_vars:
            poles = int(self.param_vars['num_poles'].get())
            current_slots = int(self.param_vars['num_slots'].get())
            
            # Calculate slots per pole
            slots_per_pole = current_slots / poles
            
            # Round to nearest multiple of 3
            slots_per_pole = max(3, round(slots_per_pole / 3) * 3)
            
            # Update slots
            new_slots = poles * slots_per_pole
            self.param_vars['num_slots'].set(new_slots)
    
    def on_window_resize(self, event):
        """Handle window resize to scale fonts"""
        # Only respond to root window resize events
        if event.widget != self.root:
            return
        
        # Calculate scale factor based on window size
        width_scale = event.width / self.base_width
        height_scale = event.height / self.base_height
        new_scale = min(width_scale, height_scale)
        
        # Only update if scale changed significantly
        if abs(new_scale - self.current_font_scale) > 0.05:
            self.current_font_scale = new_scale
            self.update_font_sizes(new_scale)
    
    def update_font_sizes(self, scale):
        """Update all font sizes based on scale factor"""
        # Limit scale to reasonable range
        scale = max(0.7, min(scale, 1.5))
        
        # Update results text font
        results_size = max(7, int(9 * scale))
        self.results_text.config(font=('Courier', results_size))
        
        # Update parameter labels
        label_size = max(8, int(10 * scale))
        for label in self.label_widgets:
            try:
                label.config(font=('TkDefaultFont', label_size))
            except:
                pass
        
        # Update title labels
        title_size = max(10, int(14 * scale))
        for label in self.title_labels:
            try:
                label.config(font=('Arial', title_size, 'bold'))
            except:
                pass
        
        # Update entry widgets
        entry_size = max(8, int(10 * scale))
        entry_width = max(8, int(10 * scale))
        for entry in self.entry_widgets:
            try:
                entry.config(font=('TkDefaultFont', entry_size), width=entry_width)
            except:
                pass
        
        # Update labelframe fonts
        frame_size = max(9, int(11 * scale))
        for frame in self.frame_widgets:
            try:
                frame.config(font=('TkDefaultFont', frame_size, 'bold'))
            except:
                pass
        
        # Update button fonts
        button_size = max(9, int(11 * scale))
        for button in self.button_widgets:
            try:
                button.config(style='Scaled.TButton')
            except:
                pass
        
        # Update combobox fonts
        combo_size = max(8, int(10 * scale))
        combo_width = max(12, int(18 * scale))
        for combo in self.combobox_widgets:
            try:
                combo.config(font=('TkDefaultFont', combo_size), width=combo_width)
            except:
                pass
        
        # Update tab fonts in notebook
        if hasattr(self, 'plot_notebook'):
            try:
                tab_size = max(9, int(11 * scale))
                style = ttk.Style()
                style.configure('TNotebook.Tab', font=('TkDefaultFont', tab_size))
            except:
                pass
        
        # Update matplotlib plot fonts
        self.update_plot_fonts(scale)
    
    def update_plot_fonts(self, scale):
        """Update matplotlib plot font sizes"""
        if not hasattr(self, 'plot_figures'):
            return
        
        # Calculate scaled font sizes
        title_size = max(10, int(14 * scale))
        label_size = max(8, int(12 * scale))
        tick_size = max(7, int(10 * scale))
        legend_size = max(7, int(9 * scale))
        
        # Update each figure
        for tab_name, fig in self.plot_figures.items():
            try:
                for ax in fig.get_axes():
                    # Update title
                    if ax.get_title():
                        ax.title.set_fontsize(title_size)
                    
                    # Update axis labels
                    ax.xaxis.label.set_fontsize(label_size)
                    ax.yaxis.label.set_fontsize(label_size)
                    
                    # Update tick labels
                    ax.tick_params(axis='both', labelsize=tick_size)
                    
                    # Update legend if present
                    legend = ax.get_legend()
                    if legend:
                        for text in legend.get_texts():
                            text.set_fontsize(legend_size)
                    
                    # Update text annotations
                    for text in ax.texts:
                        current_size = text.get_fontsize()
                        if current_size:
                            # Scale text proportionally
                            new_size = max(6, int(current_size * scale))
                            text.set_fontsize(new_size)
                    
                    # Update colorbar if present
                    if hasattr(ax, 'collections'):
                        for collection in ax.collections:
                            if hasattr(collection, 'colorbar') and collection.colorbar:
                                collection.colorbar.ax.tick_params(labelsize=tick_size)
                                if collection.colorbar.ax.yaxis.label:
                                    collection.colorbar.ax.yaxis.label.set_fontsize(label_size)
                
                # Redraw the canvas
                self.plot_canvases[tab_name].draw()
            except Exception as e:
                pass


def main():
    """Main entry point"""
    root = tk.Tk()
    
    # Configure style
    style = ttk.Style()
    style.theme_use('clam')
    
    app = MotorDesignApp(root)
    
    # Ensure clean exit when window is closed
    def on_closing():
        try:
            root.quit()
            root.destroy()
        except:
            pass
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    try:
        root.mainloop()
    finally:
        try:
            root.quit()
            root.destroy()
        except:
            pass


if __name__ == "__main__":
    main()
