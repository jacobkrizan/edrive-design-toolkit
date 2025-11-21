"""
E-Drive Parameter Viewer - Main UI
Displays powertrain parameters loaded from Excel files
"""

import sys
import os
# Add src directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from typing import Optional
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from excel_importer import ExcelParameterImporter
from part_classes import Powertrain
from display_motor import MotorDisplay, display_motor
from motor_mesher import mesh_motor


class EDriveParameterViewer:
    """Main application for viewing e-drive parameters from Excel"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("E-Drive Design Toolkit v1.0 - Powertrain Builder")
        self.root.geometry("1920x1080")
        
        # Data storage
        self.powertrain: Optional[Powertrain] = None
        self.current_file: Optional[str] = None
        self.motor_canvas = None
        self.toolbar = None
        
        self.create_ui()
        
    def create_ui(self):
        """Create the user interface"""
        
        # Menu bar
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Exit", command=self.root.quit)
        
        # Top frame for controls
        top_frame = ttk.Frame(self.root, padding="10")
        top_frame.pack(fill=tk.X)
        
        ttk.Label(top_frame, text="Excel File:").pack(side=tk.LEFT)
        self.file_label = ttk.Label(top_frame, text="No file selected", foreground="gray")
        self.file_label.pack(side=tk.LEFT, padx=10)
        
        ttk.Button(top_frame, text="Select Excel File", command=self.select_excel).pack(side=tk.RIGHT, padx=5)
        self.build_button = ttk.Button(top_frame, text="Construct Powertrain", command=self.construct_powertrain, state=tk.DISABLED)
        self.build_button.pack(side=tk.RIGHT, padx=5)
        self.mesh_button = ttk.Button(top_frame, text="Mesh Motor", command=self.mesh_motor, state=tk.DISABLED)
        self.mesh_button.pack(side=tk.RIGHT, padx=5)
        
        # Main content area - split view (left = parameters, right = motor)
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create PanedWindow for resizable split
        paned = ttk.PanedWindow(main_frame, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True)
        
        # Left Panel - Parameters (scrollable)
        left_frame = ttk.LabelFrame(paned, text="Parameters", padding=5)
        paned.add(left_frame, weight=1)
        
        # Create scrollable frame for parameters
        canvas = tk.Canvas(left_frame)
        scrollbar = ttk.Scrollbar(left_frame, orient="vertical", command=canvas.yview)
        self.main_scroll = ttk.Frame(canvas)
        
        self.main_scroll.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=self.main_scroll, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Right Panel - Motor Visualization
        right_frame = ttk.LabelFrame(paned, text="Motor Visualization", padding=5)
        paned.add(right_frame, weight=1)
        
        self.plot_frame = ttk.Frame(right_frame)
        self.plot_frame.pack(fill=tk.BOTH, expand=True)
        
        # Status bar
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill=tk.X, side=tk.BOTTOM)
        self.status_label = ttk.Label(status_frame, text="Ready - Select an Excel file to begin", relief=tk.SUNKEN, anchor=tk.W)
        self.status_label.pack(fill=tk.X, padx=5, pady=2)
        
        # Show initial message
        self.show_welcome_message()
        
    def show_welcome_message(self):
        """Display welcome message in empty UI"""
        welcome_label = ttk.Label(
            self.main_scroll, 
            text="Select an Excel parameter file and click 'Construct Powertrain' to begin",
            font=('Arial', 14),
            foreground="gray"
        )
        welcome_label.pack(expand=True, pady=200)
    
    def select_excel(self):
        """Select an Excel file"""
        filename = filedialog.askopenfilename(
            title="Select Excel Parameter File",
            filetypes=[("Excel files", "*.xlsx *.xls"), ("All files", "*.*")],
            initialdir=os.path.join(os.path.dirname(__file__), '..', 'projects')
        )
        
        if filename:
            self.current_file = filename
            self.file_label.config(text=os.path.basename(filename), foreground="black")
            self.build_button.config(state=tk.NORMAL)
            self.status_label.config(text=f"File selected: {os.path.basename(filename)}")
    
    def construct_powertrain(self):
        """Construct powertrain from Excel parameters"""
        if not self.current_file:
            messagebox.showerror("Error", "No Excel file selected")
            return
        
        try:
            self.status_label.config(text=f"Constructing powertrain from {os.path.basename(self.current_file)}...")
            self.root.update()
            
            # Import parameters and construct powertrain
            importer = ExcelParameterImporter(self.current_file)
            components = importer.import_all()
            
            if components:
                # Create powertrain with components
                self.powertrain = Powertrain()
                
                # Assign components from Excel to the powertrain's primary drive unit
                if components.get('motor'):
                    self.powertrain.primary_drive_unit.motor = components['motor']
                    self.powertrain.primary_drive_unit.motor._parent = self.powertrain.primary_drive_unit
                
                if components.get('inverter'):
                    self.powertrain.primary_drive_unit.inverter = components['inverter']
                    self.powertrain.primary_drive_unit.inverter._parent = self.powertrain.primary_drive_unit
                
                if components.get('gearbox'):
                    self.powertrain.primary_drive_unit.gearbox = components['gearbox']
                    self.powertrain.primary_drive_unit.gearbox._parent = self.powertrain.primary_drive_unit
                
                if components.get('battery'):
                    self.powertrain.battery = components['battery']
                    self.powertrain.battery._parent = self.powertrain
                
                # Display the powertrain parameters
                self.display_powertrain()
                
                # Update motor plot
                self.update_motor_plot()
                
                # Enable mesh button
                self.mesh_button.config(state=tk.NORMAL)
                
                self.status_label.config(text=f"Powertrain constructed successfully from {os.path.basename(self.current_file)}")
            else:
                messagebox.showerror("Error", "Failed to load parameters from Excel file")
                self.status_label.config(text="Construction failed")
                
        except Exception as e:
            messagebox.showerror("Error", f"Error constructing powertrain:\n{str(e)}")
            self.status_label.config(text="Error")
    
    def update_motor_plot(self):
        """Update the motor plot in the right panel"""
        if not self.powertrain:
            return
        
        try:
            # Clear existing plot
            for widget in self.plot_frame.winfo_children():
                widget.destroy()
            
            # Get motor from powertrain
            motor = self.powertrain.primary_drive_unit.motor
            
            # Create motor display
            motor_display = MotorDisplay(motor)
            fig, ax = motor_display.plot(show_dimensions=False, show_centerlines=False)
            
            # Embed matplotlib figure in tkinter
            canvas = FigureCanvasTkAgg(fig, master=self.plot_frame)
            canvas.draw()
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            
            # Add matplotlib navigation toolbar for zoom, pan, etc.
            toolbar = NavigationToolbar2Tk(canvas, self.plot_frame, pack_toolbar=False)
            toolbar.update()
            toolbar.pack(side=tk.BOTTOM, fill=tk.X)
            
            self.motor_canvas = canvas
            self.toolbar = toolbar
            
        except Exception as e:
            error_label = ttk.Label(self.plot_frame, 
                                   text=f"Error generating motor plot:\n{str(e)}",
                                   foreground="red")
            error_label.pack(expand=True)
    
    def plot_motor(self):
        """Plot motor cross-section geometry"""
        if not self.powertrain:
            messagebox.showerror("Error", "No powertrain constructed")
            return
        
        try:
            self.status_label.config(text="Generating motor plot...")
            self.root.update()
            
            # Get motor from powertrain
            motor = self.powertrain.primary_drive_unit.motor
            
            # Generate plot
            display_motor(motor, show=True, save_path='motor_cross_section.png')
            
            self.status_label.config(text="Motor plot generated successfully")
            
        except Exception as e:
            messagebox.showerror("Error", f"Error generating plot:\n{str(e)}")
            self.status_label.config(text="Plot error")
    
    def mesh_motor(self):
        """Generate motor mesh and open in gmsh viewer"""
        if not self.powertrain:
            messagebox.showerror("Error", "No powertrain constructed")
            return
        
        try:
            self.status_label.config(text="Generating motor mesh...")
            self.root.update()
            
            # Get motor from powertrain
            motor = self.powertrain.primary_drive_unit.motor
            
            # Generate mesh and open viewer
            mesh_file = mesh_motor(motor, mesh_size=2.0, show_viewer=True)
            
            self.status_label.config(text=f"Motor mesh generated: {os.path.basename(mesh_file)}")
            
        except Exception as e:
            messagebox.showerror("Error", f"Error generating mesh:\n{str(e)}")
            self.status_label.config(text="Mesh generation error")
    
    def display_powertrain(self):
        """Display all powertrain parameters in a single list"""
        if not self.powertrain:
            return
        
        # Clear existing widgets
        for widget in self.main_scroll.winfo_children():
            widget.destroy()
        
        # Title
        title_label = ttk.Label(self.main_scroll, text="Powertrain Parameters", font=('Arial', 16, 'bold'))
        title_label.pack(anchor=tk.W, pady=10, padx=10)
        
        # Create single frame for all parameters
        param_frame = ttk.Frame(self.main_scroll)
        param_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Configure grid columns
        param_frame.columnconfigure(0, weight=0)  # Component name
        param_frame.columnconfigure(1, weight=0)  # Parameter name
        param_frame.columnconfigure(2, weight=1)  # Parameter value
        
        row = 0
        
        # Helper function to add parameters
        def add_parameters(component_name: str, component, row_start: int) -> int:
            """Add all parameters from a component to the list"""
            current_row = row_start
            
            for attr_name in dir(component):
                if attr_name.startswith('_') or callable(getattr(component, attr_name)):
                    continue
                
                value = getattr(component, attr_name)
                
                # Skip None values and complex objects (except simple nested components)
                if value is None:
                    continue
                
                # Skip nested component objects (we'll handle them separately)
                if hasattr(value, '__dict__') and not isinstance(value, (str, int, float, bool)):
                    continue
                
                # Component label (only on first parameter)
                if current_row == row_start:
                    comp_label = ttk.Label(param_frame, text=f"{component_name}:", 
                                          font=('Arial', 10, 'bold'), foreground="navy")
                    comp_label.grid(row=current_row, column=0, sticky=tk.W, padx=(5, 15), pady=2)
                
                # Parameter name
                param_label = ttk.Label(param_frame, text=f"  {attr_name}:", font=('Arial', 10))
                param_label.grid(row=current_row, column=1, sticky=tk.W, padx=5, pady=2)
                
                # Parameter value
                value_label = ttk.Label(param_frame, text=str(value), font=('Arial', 10, 'bold'))
                value_label.grid(row=current_row, column=2, sticky=tk.W, padx=10, pady=2)
                
                current_row += 1
            
            return current_row
        
        # Display Primary Drive Unit
        drive_unit = self.powertrain.primary_drive_unit
        
        # Motor and sub-components
        if drive_unit.motor:
            motor = drive_unit.motor
            row = add_parameters("Motor", motor, row)
            
            if motor.stator:
                row = add_parameters("Stator", motor.stator, row)
                if motor.stator.core:
                    row = add_parameters("Stator Core", motor.stator.core, row)
                if motor.stator.windings:
                    row = add_parameters("Stator Windings", motor.stator.windings, row)
                if motor.stator.slot_liner:
                    row = add_parameters("Slot Liner", motor.stator.slot_liner, row)
            
            if motor.rotor:
                if motor.rotor.core:
                    row = add_parameters("Rotor Core", motor.rotor.core, row)
                if motor.rotor.magnet:
                    row = add_parameters("Magnet", motor.rotor.magnet, row)
                if motor.rotor.shaft:
                    row = add_parameters("Shaft", motor.rotor.shaft, row)
        
        # Inverter
        if drive_unit.inverter:
            row = add_parameters("Inverter", drive_unit.inverter, row)
        
        # Gearbox
        if drive_unit.gearbox:
            row = add_parameters("Gearbox", drive_unit.gearbox, row)
        
        # Battery
        if self.powertrain.battery:
            row = add_parameters("Battery", self.powertrain.battery, row)
        
        # Summary section
        separator = ttk.Separator(param_frame, orient='horizontal')
        separator.grid(row=row, column=0, columnspan=3, sticky='ew', pady=10)
        row += 1
        
        summary_label = ttk.Label(param_frame, text="Powertrain Summary:", 
                                 font=('Arial', 12, 'bold'), foreground="darkgreen")
        summary_label.grid(row=row, column=0, columnspan=3, sticky=tk.W, padx=5, pady=5)
        row += 1
        
        # Total mass
        mass = self.powertrain.compute_mass()
        mass_label = ttk.Label(param_frame, text=f"  Total Mass:", font=('Arial', 10))
        mass_label.grid(row=row, column=1, sticky=tk.W, padx=5, pady=2)
        mass_value = ttk.Label(param_frame, text=f"{mass:.2f} kg", font=('Arial', 10, 'bold'))
        mass_value.grid(row=row, column=2, sticky=tk.W, padx=10, pady=2)
        row += 1
        
        # Total cost
        cost = self.powertrain.compute_cost()
        cost_label = ttk.Label(param_frame, text=f"  Total Cost:", font=('Arial', 10))
        cost_label.grid(row=row, column=1, sticky=tk.W, padx=5, pady=2)
        cost_value = ttk.Label(param_frame, text=f"${cost:.2f}", font=('Arial', 10, 'bold'))
        cost_value.grid(row=row, column=2, sticky=tk.W, padx=10, pady=2)


def main():
    """Main entry point"""
    root = tk.Tk()
    app = EDriveParameterViewer(root)
    root.mainloop()


if __name__ == "__main__":
    main()
