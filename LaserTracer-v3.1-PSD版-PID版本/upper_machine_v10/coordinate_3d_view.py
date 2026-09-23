from __future__ import annotations

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget


def _xyz(value):
    point = np.asarray(value, dtype=float).reshape(-1)
    if point.size < 3 or not np.all(np.isfinite(point[:3])):
        raise ValueError(f"Invalid 3D point: {value}")
    return point[:3]


def _format_point(point):
    point = _xyz(point)
    return f"({point[0]:.4f}, {point[1]:.4f}, {point[2]:.4f})"


class Coordinate3DView(QWidget):
    """Event-driven 3D scene in the femtosecond-origin coordinate frame."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)

        self.summary_label = QLabel(self)
        self.summary_label.setWordWrap(True)
        self.summary_label.setStyleSheet("font-size: 11pt;")
        layout.addWidget(self.summary_label)

        self.figure = Figure(figsize=(8.0, 6.0), tight_layout=True)
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas, 1)

        self.transmitter_positions = {}
        self.solution = None
        self.station_source = ""
        self.update_scene({})

    def update_scene(
        self,
        transmitter_positions_local_mm,
        solution_local_mm=None,
        station_source="",
    ):
        self.transmitter_positions = {
            int(transmitter_id): _xyz(position).copy()
            for transmitter_id, position in transmitter_positions_local_mm.items()
        }
        self.solution = (
            None if solution_local_mm is None else _xyz(solution_local_mm).copy()
        )
        self.station_source = str(station_source or "")
        self._update_summary()
        self._draw()

    def _update_summary(self):
        if self.solution is None:
            self.summary_label.setText("<b>测量点坐标：</b>等待解算")
        else:
            self.summary_label.setText(
                "<b>测量点坐标：</b>"
                f"X={self.solution[0]:.4f}, "
                f"Y={self.solution[1]:.4f}, "
                f"Z={self.solution[2]:.4f} mm"
            )

    def _draw(self):
        self.figure.clear()
        axis = self.figure.add_subplot(111, projection="3d")

        origin = np.zeros(3, dtype=float)
        plotted_points = [origin]
        axis.scatter(
            [0.0], [0.0], [0.0],
            color="#d62728", marker="*", s=150,
            label="Femtosecond origin",
        )
        axis.text(0.0, 0.0, 0.0, "  O", color="#d62728")

        station_points = []
        colors = ("#1f77b4", "#17becf", "#9467bd", "#8c564b")
        for index, transmitter_id in enumerate(sorted(self.transmitter_positions)):
            point = self.transmitter_positions[transmitter_id]
            station_points.append(point)
            plotted_points.append(point)
            color = colors[index % len(colors)]
            axis.scatter(
                [point[0]], [point[1]], [point[2]],
                color=color, marker="^", s=80,
                label=f"wMPS TX {transmitter_id}",
            )
            axis.text(
                point[0], point[1], point[2],
                f"  TX{transmitter_id}", color=color,
            )

        if len(station_points) >= 2:
            station_array = np.vstack(station_points)
            axis.plot(
                station_array[:, 0],
                station_array[:, 1],
                station_array[:, 2],
                color="#7f7f7f", linestyle="--", linewidth=1.0,
            )

        if self.solution is not None:
            point = self.solution
            plotted_points.append(point)
            axis.scatter(
                [point[0]], [point[1]], [point[2]],
                color="#2ca02c", marker="o", s=110,
                label="Solved corner cube",
            )
            axis.text(point[0], point[1], point[2], "  Result", color="#2ca02c")
            axis.plot(
                [0.0, point[0]],
                [0.0, point[1]],
                [0.0, point[2]],
                color="#2ca02c", linewidth=1.2, alpha=0.75,
            )

        points = np.vstack(plotted_points)
        center = (np.min(points, axis=0) + np.max(points, axis=0)) / 2.0
        radius = max(float(np.max(np.ptp(points, axis=0))) / 2.0, 100.0)
        padding = radius * 1.12
        axis.set_xlim(center[0] - padding, center[0] + padding)
        axis.set_ylim(center[1] - padding, center[1] + padding)
        axis.set_zlim(center[2] - padding, center[2] + padding)
        axis.set_box_aspect((1.0, 1.0, 1.0))

        axis.set_xlabel("X / mm")
        axis.set_ylabel("Y / mm")
        axis.set_zlabel("Z / mm")
        axis.set_title("Femtosecond-origin coordinate frame (wMPS axis directions)")
        axis.view_init(elev=24, azim=-58)
        axis.grid(True, alpha=0.35)
        axis.legend(loc="upper right")
        self.canvas.draw_idle()
