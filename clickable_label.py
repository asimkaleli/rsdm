# -*- coding: utf-8 -*-
from PySide2.QtWidgets import QLabel
from PySide2.QtCore import Qt, Signal
from PySide2.QtGui import QPainter, QPen, QColor, QCursor

class ClickableLabel(QLabel):
    clicked = Signal(int, int)
    selectionCompleted = Signal(str, int, int)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.markers = []              # [(row_index, x, y)]
        self.pixmap_orig = None
        self.select_mode = None        # None | 'first' | 'last'
        self.first_point = None
        self.last_point = None
        self.area_points = {}          # {"A"|"B"|"C"|"D": (x, y)}

    def start_select_first(self):
        self.select_mode = 'first'
        self.setCursor(QCursor(Qt.CrossCursor))

    def start_select_last(self):
        self.select_mode = 'last'
        self.setCursor(QCursor(Qt.CrossCursor))

    def start_select_area(self, corner):
        corner = str(corner).upper()
        if corner not in ("A", "B", "C", "D"):
            raise ValueError("Area corner must be A, B, C or D.")
        self.select_mode = f"area:{corner}"
        self.setCursor(QCursor(Qt.CrossCursor))

    def clear_selection(self):
        self.select_mode = None
        self.first_point = None
        self.last_point = None
        self.area_points.clear()
        self.setCursor(QCursor(Qt.ArrowCursor))
        self.update()

    def setPixmap(self, pixmap):
        super().setPixmap(pixmap)
        if pixmap and not pixmap.isNull():
            self.pixmap_orig = pixmap
        self.update()

    def add_marker(self, row_index, x, y):
        self.markers.append((row_index, x, y))
        self.update()

    def remove_marker(self, row_index):
        self.markers = [m for m in self.markers if m[0] != row_index]
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        pen = QPen(QColor("red")); pen.setWidth(2)
        p.setPen(pen); p.setBrush(QColor("red"))
        for idx, x, y in self.markers:
            r = 5
            p.drawEllipse(x - r, y - r, 2*r, 2*r)
            p.setPen(QColor("red"))
            p.drawText(x + r + 2, y + r + 2, str(idx + 1))
            p.setPen(pen)
        r = 5
        if self.first_point:
            fx, fy = self.first_point
            if all(not (fx == mx and fy == my) for _, mx, my in self.markers):
                p.setBrush(QColor("green")); p.setPen(QColor("green"))
                p.drawEllipse(fx - r, fy - r, 2*r, 2*r)
        if self.last_point:
            lx, ly = self.last_point
            if all(not (lx == mx and ly == my) for _, mx, my in self.markers):
                p.setBrush(QColor("blue")); p.setPen(QColor("blue"))
                p.drawEllipse(lx - r, ly - r, 2*r, 2*r)
        area_colors = {
            "A": QColor("#00a86b"), "B": QColor("#00a86b"),
            "C": QColor("#ff8c00"), "D": QColor("#ff8c00"),
        }
        for corner, (cx, cy) in self.area_points.items():
            color = area_colors.get(corner, QColor("magenta"))
            p.setBrush(color); p.setPen(color)
            p.drawEllipse(cx - r, cy - r, 2*r, 2*r)
            p.drawText(cx + r + 3, cy - r - 2, corner)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            x, y = event.pos().x(), event.pos().y()
            if self.select_mode in ('first', 'last'):
                completed_mode = self.select_mode
                if completed_mode == 'first': self.first_point = (x, y)
                else:                         self.last_point  = (x, y)
                self.select_mode = None
                self.setCursor(QCursor(Qt.ArrowCursor))
                self.update()
                self.selectionCompleted.emit(completed_mode, x, y)
                return
            if self.select_mode and self.select_mode.startswith('area:'):
                corner = self.select_mode.split(':', 1)[1]
                self.area_points[corner] = (x, y)
                self.select_mode = None
                self.setCursor(QCursor(Qt.ArrowCursor))
                self.update()
                self.selectionCompleted.emit(f"area:{corner}", x, y)
                return
            for row_index, mx, my in self.markers:
                if (x - mx) ** 2 + (y - my) ** 2 <= 10 ** 2:
                    return
            self.clicked.emit(x, y)
