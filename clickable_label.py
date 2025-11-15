# -*- coding: utf-8 -*-
from PySide2.QtWidgets import QLabel, QTableWidgetItem
from PySide2.QtCore import Qt, Signal
from PySide2.QtGui import QPainter, QPen, QColor, QCursor

class ClickableLabel(QLabel):
    clicked = Signal(int, int)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.markers = []              # [(row_index, x, y)]
        self.pixmap_orig = None
        self.dragging_index = None
        self.table_ref = None
        self.select_mode = None        # None | 'first' | 'last'
        self.first_point = None
        self.last_point = None

    def start_select_first(self):
        self.select_mode = 'first'
        self.setCursor(QCursor(Qt.CrossCursor))

    def start_select_last(self):
        self.select_mode = 'last'
        self.setCursor(QCursor(Qt.CrossCursor))

    def clear_selection(self):
        self.select_mode = None
        self.first_point = None
        self.last_point = None
        self.setCursor(QCursor(Qt.ArrowCursor))
        self.update()

    def set_table(self, table):
        self.table_ref = table

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
        if not self.pixmap():
            return
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

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            x, y = event.pos().x(), event.pos().y()
            if self.select_mode in ('first', 'last'):
                if self.select_mode == 'first': self.first_point = (x, y)
                else:                           self.last_point  = (x, y)
                self.select_mode = None
                self.setCursor(QCursor(Qt.ArrowCursor))
                self.update()
                return
            for i, (row_index, mx, my) in enumerate(self.markers):
                if (x - mx) ** 2 + (y - my) ** 2 <= 10 ** 2:
                    self.dragging_index = i
                    self.setCursor(QCursor(Qt.ClosedHandCursor))
                    return
            self.clicked.emit(x, y)

    def mouseMoveEvent(self, event):
        if self.dragging_index is not None and event.buttons() & Qt.LeftButton:
            x, y = event.pos().x(), event.pos().y()
            row_index, old_x, old_y = self.markers[self.dragging_index]
            self.markers[self.dragging_index] = (row_index, x, y)
            if self.first_point == (old_x, old_y): self.first_point = (x, y)
            if self.last_point  == (old_x, old_y): self.last_point  = (x, y)
            self.update()
            if self.table_ref:
                self.table_ref.setItem(row_index, 0, QTableWidgetItem(str(x)))
                self.table_ref.setItem(row_index, 1, QTableWidgetItem(str(y)))

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging_index = None
            self.setCursor(QCursor(Qt.ArrowCursor))
