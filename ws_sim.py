#!/usr/bin/env python3
"""ws_sim — standalone 3D robot viewer.

Runs the VTK viewer from nextup_hmi in its OWN process so the main HMI never
pays for it. Two reasons this is a separate process rather than a widget:

  1. nextup_hmi is PyQt6; the main HMI is PyQt5. Importing both into one
     process links two Qt builds and crashes.
  2. VTK plus a TF listener costs real CPU. Killing the process is the only
     way to reliably reclaim all of it — hiding a widget still leaves the
     render loop, the TF buffer and the ROS executor alive.

Launch:   python3 ws_sim.py [--urdf PATH] [--meshes DIR]
Stop:     SIGTERM (the parent sends this) or close the window.

Expects nextup_hmi on PYTHONPATH, or set NEXTUP_HMI_DIR.
"""

import argparse
import os
import signal
import sys
import threading

HMI_DIR = os.environ.get(
    "NEXTUP_HMI_DIR", os.path.expanduser("~/nextup_hmi"))
if os.path.isdir(HMI_DIR) and HMI_DIR not in sys.path:
    sys.path.insert(0, HMI_DIR)

try:
    from PyQt6.QtWidgets import QApplication, QMainWindow
    from PyQt6.QtCore import QTimer, Qt
except ImportError as exc:
    sys.stderr.write(f"ws_sim: PyQt6 is required ({exc})\n")
    sys.exit(2)

import rclpy
from rclpy.node import Node

try:
    from viewer_widget import Robot3DViewer
except ImportError as exc:
    sys.stderr.write(
        f"ws_sim: could not import viewer_widget from {HMI_DIR} ({exc})\n"
        "Set NEXTUP_HMI_DIR to the nextup_hmi checkout.\n")
    sys.exit(2)


class WsSimWindow(QMainWindow):
    def __init__(self, urdf_path, mesh_path):
        super().__init__()
        self.setWindowTitle("ws_sim — Nextup Cobot")
        self.resize(900, 700)

        # Create the viewer widget
        # Robot3DViewer internally contains QVTKRenderWindowInteractor
        self.viewer = Robot3DViewer(
            urdf_path=urdf_path, mesh_path=mesh_path)
        self.setCentralWidget(self.viewer)

        # Configure VTK interaction - Qt owns the event loop
        self._setup_vtk_interaction()

        # Initialize ROS
        rclpy.init(args=None)
        self.node = Node("ws_sim_viewer")

        # Setup TF listener in the viewer
        if hasattr(self.viewer, "setup_tf"):
            self.viewer.setup_tf(self.node)

        # ROS spinning thread (separate from Qt's event loop)
        self._stop = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        # Timer for TF updates (Qt-owned, runs in Qt's event loop)
        self._tf_timer = QTimer(self)
        self._tf_timer.timeout.connect(self.update_from_tf)
        self._tf_timer.start(50)  # 20 Hz

        # Set up keyboard shortcuts
        self._setup_keyboard_shortcuts()

    def _setup_vtk_interaction(self):
        """Configure VTK interaction without taking over the event loop.

        Key points:
        - QVTKRenderWindowInteractor is already integrated with Qt
        - The interactor is initialized but NOT Start() called
        - Qt's event loop drives both UI and VTK events
        - TrackballCamera style provides RViz-like controls
        """
        # Get the VTK widget from the viewer
        vtk_widget = getattr(self.viewer, "vtk_widget", None)
        if vtk_widget is None:
            # Try to find it by searching children
            for child in self.viewer.children():
                if hasattr(child, "GetRenderWindow"):
                    vtk_widget = child
                    break

        if vtk_widget is None:
            sys.stderr.write("ws_sim: Could not find VTK widget\n")
            return

        # Get render window
        render_window = vtk_widget.GetRenderWindow()
        if render_window is None:
            sys.stderr.write("ws_sim: No render window\n")
            return

        # Get or create interactor
        interactor = render_window.GetInteractor()
        if interactor is None:
            from vtkmodules.vtkRenderingCore import vtkRenderWindowInteractor
            interactor = vtkRenderWindowInteractor()
            render_window.SetInteractor(interactor)

        # Set trackball camera style for RViz-like controls
        from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
        style = vtkInteractorStyleTrackballCamera()
        interactor.SetInteractorStyle(style)

        # Initialize the interactor (but DO NOT call Start())
        # Qt will drive the event loop via the QVTK widget
        interactor.Initialize()

        # Enable focus for keyboard events
        vtk_widget.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        vtk_widget.setFocus(Qt.FocusReason.ActiveWindowFocusReason)

        # Force an initial render
        render_window.Render()

        sys.stderr.write("ws_sim: VTK interaction configured (TrackballCamera)\n")

    def _setup_keyboard_shortcuts(self):
        """Set up keyboard shortcuts for camera control."""
        # Get the VTK widget for keyboard events
        vtk_widget = getattr(self.viewer, "vtk_widget", None)
        if vtk_widget is None:
            for child in self.viewer.children():
                if hasattr(child, "GetRenderWindow"):
                    vtk_widget = child
                    break

        if vtk_widget is None:
            return

        # Get render window and interactor
        render_window = vtk_widget.GetRenderWindow()
        if render_window is None:
            return

        interactor = render_window.GetInteractor()
        if interactor is None:
            return

        # Override key press to add shortcuts
        original_on_key = interactor.OnKeyPress

        def on_key_press():
            """Handle keyboard shortcuts."""
            key = interactor.GetKeySym()
            renderer = interactor.GetRenderWindow().GetRenderers().GetFirstRenderer()

            if key == "r" or key == "R":
                # Reset camera
                if renderer:
                    renderer.ResetCamera()
                    render_window.Render()
            elif key == "f" or key == "F":
                # Fit camera to scene
                if renderer:
                    renderer.ResetCamera()
                    render_window.Render()
            else:
                # Pass through to original handler
                original_on_key()

        # Replace the OnKeyPress method
        interactor.OnKeyPress = on_key_press

    def update_from_tf(self):
        """Update the viewer from TF data."""
        if hasattr(self.viewer, "update_from_tf"):
            self.viewer.update_from_tf()

        # Ensure the view updates
        vtk_widget = getattr(self.viewer, "vtk_widget", None)
        if vtk_widget:
            render_window = vtk_widget.GetRenderWindow()
            if render_window:
                render_window.Render()

    def _spin(self):
        """ROS spin loop - runs in separate thread."""
        while not self._stop.is_set() and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception as e:
                sys.stderr.write(f"ws_sim: ROS spin error: {e}\n")
                break

    def closeEvent(self, event):
        """Clean shutdown."""
        # Signal the ROS thread to stop
        self._stop.set()

        # Stop the TF timer
        try:
            self._tf_timer.stop()
        except Exception:
            pass

        # Clean up VTK (the widget will be destroyed by Qt)
        try:
            vtk_widget = getattr(self.viewer, "vtk_widget", None)
            if vtk_widget:
                render_window = vtk_widget.GetRenderWindow()
                if render_window:
                    interactor = render_window.GetInteractor()
                    if interactor:
                        interactor.TerminateApp()
        except Exception:
            pass

        # Clean up ROS
        try:
            self.node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description="Standalone ws_sim viewer")
    parser.add_argument("--urdf", default=None, help="Path to URDF file")
    parser.add_argument("--meshes", default=None, help="Path to mesh directory")
    args = parser.parse_args()

    # Create Qt application
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Create and show the viewer window
    window = WsSimWindow(args.urdf, args.meshes)
    window.show()

    # Signal handlers for clean shutdown
    def _term(_signum, _frame):
        window.close()
        app.quit()

    signal.signal(signal.SIGTERM, _term)
    signal.signal(signal.SIGINT, _term)

    # Start Qt's event loop - this owns both UI and VTK events
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
