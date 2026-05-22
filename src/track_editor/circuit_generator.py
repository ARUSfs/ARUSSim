#!/usr/bin/env python3
"""
circuit_generator.py — ARUS Andalucia Racing Team v2

PyQt5 GUI — vintage industrial style, readable, square.
"""
import sys, math
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSpinBox, QDoubleSpinBox, QScrollArea,
    QFrame, QProgressBar, QDialog, QFileDialog, QCheckBox,
    QGroupBox, QSizePolicy, QMessageBox, QStatusBar, QGridLayout,
)
from PyQt5.QtCore  import Qt, QThread, pyqtSignal, QRectF, QPointF, QSize, QTimer
from PyQt5.QtGui   import (
    QPainter, QPen, QBrush, QColor, QFont, QPainterPath,
    QPixmap, QCursor, QPalette,
)
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from circuit_gen_core import (
    generate_circuit, export_pcd, export_json, export_both,
    new_session, _next_seed,
    RULE_LENGTH_MIN, RULE_LENGTH_MAX, RULE_WIDTH_MIN,
    RULE_OUTER_D_MIN, RULE_CONE_GAP_MAX,
    _HAS_PROFILES, _HAS_MAPFILE,
)

# Palette — vintage industrial / oscilloscope
C_BG        = QColor(18,  18,  18)   # main background
C_PANEL     = QColor(24,  24,  24)   # panel lateral
C_CARD      = QColor(28,  28,  28)   # card
C_BORDER    = QColor(55,  55,  55)   # borde normal
C_BORDER_HI = QColor(90,  90,  90)   # borde hover
C_TRACK     = QColor(36,  36,  36)   # track surface
C_TEXT      = QColor(200, 200, 200)  # texto principal
C_TEXT_DIM  = QColor(110, 110, 110)  # texto secundario
C_TEXT_FAINT= QColor(60,  60,  60)   # very faint text
C_AMBER     = QColor(210, 155, 55)   # amber accent
C_GREEN     = QColor(100, 175, 100)  # OK
C_RED       = QColor(190, 80,  80)   # FAIL
C_BLUE_CONE = QColor(80,  130, 210)  # blue cone (dimmed)
C_YELL_CONE = QColor(200, 165, 50)   # yellow cone (dimmed)
C_ORANGE    = QColor(200, 105, 40)   # orange

# Global QSS — no border-radius, monospace, all square
QSS = """
QMainWindow, QWidget, QDialog {
    background: #121212;
    color: #c8c8c8;
    font-family: "Courier New", "DejaVu Sans Mono", monospace;
}
QScrollArea  { border: none; background: #121212; }
QScrollBar:vertical   { background:#181818; width:8px; border:none; }
QScrollBar::handle:vertical { background:#383838; min-height:20px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height:0; }

QStatusBar {
    background: #0e0e0e;
    color: #505050;
    font-family: "Courier New", monospace;
    font-size: 10px;
    border-top: 1px solid #2a2a2a;
}

QGroupBox {
    font-family: "Courier New", monospace;
    font-size: 10px;
    color: #555555;
    letter-spacing: 2px;
    border: 1px solid #303030;
    margin-top: 14px;
    padding-top: 12px;
    text-transform: uppercase;
}
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }

QSpinBox, QDoubleSpinBox {
    background: #1a1a1a;
    border: 1px solid #383838;
    color: #d8d8d8;
    font-family: "Courier New", monospace;
    font-size: 12px;
    padding: 4px 6px;
    selection-background-color: #3a3a3a;
}
QSpinBox:focus, QDoubleSpinBox:focus { border-color: #888888; }
QSpinBox::up-button, QSpinBox::down-button,
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
    background: #252525; border: none; width: 14px;
}

QPushButton {
    background: #1e1e1e;
    border: 1px solid #404040;
    color: #b0b0b0;
    font-family: "Courier New", monospace;
    font-size: 11px;
    letter-spacing: 1px;
    padding: 7px 14px;
}
QPushButton:hover    { background: #2a2a2a; border-color: #707070; color: #e0e0e0; }
QPushButton:pressed  { background: #141414; }
QPushButton:disabled { color: #363636; border-color: #252525; background: #161616; }

QPushButton#btn_gen {
    background: #1a1a1a;
    border: 1px solid #c89030;
    color: #d09030;
    font-size: 13px;
    font-weight: bold;
    letter-spacing: 3px;
    padding: 10px 20px;
}
QPushButton#btn_gen:hover    { background: #252010; border-color: #e0a840; color: #e0a840; }
QPushButton#btn_gen:disabled { background: #141414; border-color: #303020; color: #404030; }

QProgressBar {
    background: #141414;
    border: 1px solid #2a2a2a;
    height: 6px;
    text-align: center;
}
QProgressBar::chunk { background: #c89030; }

QCheckBox { color: #808080; font-size: 10px; spacing: 6px; }
QCheckBox::indicator {
    width: 12px; height: 12px;
    border: 1px solid #404040; background: #1a1a1a;
}
QCheckBox::indicator:checked { background: #c89030; border-color: #c89030; }
QCheckBox:hover { color: #b0b0b0; }

QLabel { color: #c8c8c8; }
"""


#  GENERATOR THREAD
class GeneratorThread(QThread):
    circuit_ready = pyqtSignal(dict, int)
    progress      = pyqtSignal(int, int)
    finished_all  = pyqtSignal(int, int)

    def __init__(self, count, base_seed, params):
        super().__init__()
        self.count=count; self.base_seed=base_seed; self.params=params; self._stop=False

    def stop(self): self._stop=True

    def run(self):
        ok=0
        for i in range(self.count):
            if self._stop: break
            seed = _next_seed(self.base_seed, i)
            c = generate_circuit(seed,
                target_length=self.params.get('length'),
                track_width  =self.params.get('width'),
                n_ctrl_pts   =self.params.get('n_ctrl'),
                cone_spacing =self.params.get('spacing'),
            )
            c['index']=i
            if c['compliant']: ok+=1
            self.circuit_ready.emit(c, i)
            self.progress.emit(i+1, self.count)
        self.finished_all.emit(ok, self.count)


#  CIRCUIT CARD
class CircuitCard(QWidget):
    clicked = pyqtSignal(dict)
    W, H, BAR = 252, 178, 26

    def __init__(self, circuit=None, parent=None):
        super().__init__(parent)
        self.circuit=circuit; self._hov=False
        self.setFixedSize(self.W, self.H)
        self.setCursor(QCursor(Qt.PointingHandCursor))

    def set_circuit(self, c): self.circuit=c; self.update()

    def _transform(self, outer, inner, pad, W, H):
        pts = np.vstack([outer, inner])
        x0,x1 = pts[:,0].min(), pts[:,0].max()
        y0,y1 = pts[:,1].min(), pts[:,1].max()
        rx,ry = max(x1-x0,1), max(y1-y0,1)
        sc = min((W-2*pad)/rx, (H-2*pad)/ry)
        ox = pad+(W-2*pad-rx*sc)/2 - x0*sc
        oy = pad+(H-2*pad-ry*sc)/2 - y0*sc
        return lambda p: QPointF(p[0]*sc+ox, p[1]*sc+oy)

    def _poly(self, arr, T):
        path=QPainterPath(); pts=[T(p) for p in arr]
        path.moveTo(pts[0])
        for p in pts[1:]: path.lineTo(p)
        path.closeSubpath(); return path

    def paintEvent(self, _):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        W,H=self.W, self.H-self.BAR

        # background
        p.fillRect(0,0,self.W,self.H, C_CARD)
        p.setPen(QPen(C_BORDER if not self._hov else C_BORDER_HI))
        p.drawRect(0,0,self.W-1,self.H-1)

        if not self.circuit:
            p.setPen(C_TEXT_DIM); p.drawText(QRectF(0,0,self.W,H), Qt.AlignCenter,"...")
            p.end(); return

        c=self.circuit
        T=self._transform(c['outer'],c['inner'],7,W,H)

        # surface (odd-even)
        tp=QPainterPath(); tp.addPath(self._poly(c['outer'],T)); tp.addPath(self._poly(c['inner'],T))
        tp.setFillRule(Qt.OddEvenFill)
        p.fillPath(tp, QBrush(C_TRACK))

        # borders
        for arr,col,a in [(c['outer'],C_BLUE_CONE,90),(c['inner'],C_YELL_CONE,90)]:
            pen=QPen(QColor(col.red(),col.green(),col.blue(),a)); pen.setWidthF(0.8)
            p.setPen(pen); p.drawPath(self._poly(arr,T))

        # cones
        for col,cones,cr in [(C_BLUE_CONE,c['blue'],1.6),(C_YELL_CONE,c['yellow'],1.6)]:
            p.setPen(Qt.NoPen); p.setBrush(QBrush(col))
            for cone in cones:
                pt=T(cone); p.drawEllipse(pt,cr,cr)

        # big orange at start/finish
        p.setBrush(QBrush(C_ORANGE)); p.setPen(Qt.NoPen)
        p.drawEllipse(T(c['outer'][0]),4,4)

        # bottom bar
        ok=c['compliant']; by=H
        p.fillRect(0,by,self.W,self.BAR, QColor(22,22,22))
        pen=QPen(C_BORDER); p.setPen(pen); p.drawLine(0,by,self.W,by)

        fnt=QFont("Courier New"); fnt.setPixelSize(9); p.setFont(fnt)

        p.setPen(C_TEXT_DIM)
        p.drawText(QRectF(6,by,75,self.BAR), Qt.AlignVCenter, f"#{c['seed']}")
        p.drawText(QRectF(self.W//2-55,by,110,self.BAR), Qt.AlignVCenter|Qt.AlignHCenter,
                   f"{c['params']['length']:.0f} m · {c['params']['n_blue']+c['params']['n_yellow']} cones")
        p.setPen(C_GREEN if ok else C_RED)
        p.drawText(QRectF(self.W-50,by,44,self.BAR), Qt.AlignVCenter|Qt.AlignRight,
                   "OK" if ok else "FAIL")

        # amber border on hover
        if self._hov:
            pen=QPen(C_AMBER if ok else C_RED); pen.setWidth(1)
            p.setPen(pen); p.drawRect(0,0,self.W-1,self.H-1)
        p.end()

    def enterEvent(self,_): self._hov=True;  self.update()
    def leaveEvent(self,_): self._hov=False; self.update()
    def mousePressEvent(self,e):
        if e.button()==Qt.LeftButton and self.circuit: self.clicked.emit(self.circuit)


#  LARGE CANVAS (detail dialog)
class DetailCanvas(QWidget):
    def __init__(self, circuit, parent=None):
        super().__init__(parent); self.circuit=circuit; self.setMinimumSize(480,380)

    def _transform(self, outer, inner, pad):
        W,H=self.width(),self.height()
        pts=np.vstack([outer,inner])
        x0,x1=pts[:,0].min(),pts[:,0].max(); y0,y1=pts[:,1].min(),pts[:,1].max()
        rx,ry=max(x1-x0,1),max(y1-y0,1)
        sc=min((W-2*pad)/rx,(H-2*pad)/ry)
        ox=pad+(W-2*pad-rx*sc)/2-x0*sc; oy=pad+(H-2*pad-ry*sc)/2-y0*sc
        return lambda pt: QPointF(pt[0]*sc+ox, pt[1]*sc+oy)

    def _poly(self,arr,T):
        path=QPainterPath(); pts=[T(p) for p in arr]
        path.moveTo(pts[0])
        for p in pts[1:]: path.lineTo(p)
        path.closeSubpath(); return path

    def paintEvent(self,_):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        W,H=self.width(),self.height()
        p.fillRect(0,0,W,H,C_BG)
        c=self.circuit
        T=self._transform(c['outer'],c['inner'],26)

        # surface
        tp=QPainterPath(); tp.addPath(self._poly(c['outer'],T)); tp.addPath(self._poly(c['inner'],T))
        tp.setFillRule(Qt.OddEvenFill); p.fillPath(tp,QBrush(C_TRACK))

        # centerline colored by curvature
        cl=c['center']; n=len(cl); step=max(1,n//300)
        for i in range(0,n-step,step):
            p1=cl[i]; p2=cl[(i+step)%n]; pm=cl[(i-step)%n]
            ax=p2[0]-p1[0]; ay=p2[1]-p1[1]
            bx=p1[0]-pm[0]; by=p1[1]-pm[1]
            cross=abs(ax*by-ay*bx); den=math.hypot(ax,ay)*math.hypot(bx,by)
            k=cross/den if den>1e-6 else 0; t=min(k/0.5,1.0)
            r=int(80+t*120); g=int(160-t*120); b=int(210-t*180)
            pen=QPen(QColor(r,g,b,60)); pen.setWidthF(1.5); p.setPen(pen)
            p.drawLine(T(p1),T(p2))

        # borders
        for arr,col,a in [(c['outer'],C_BLUE_CONE,160),(c['inner'],C_YELL_CONE,160)]:
            pen=QPen(QColor(col.red(),col.green(),col.blue(),a)); pen.setWidthF(1.0)
            p.setPen(pen); p.drawPath(self._poly(arr,T))

        # cones
        for col,cones,cr in [(C_BLUE_CONE,c['blue'],3.2),(C_YELL_CONE,c['yellow'],3.2)]:
            p.setBrush(QBrush(col)); p.setPen(Qt.NoPen)
            for cone in cones: pt=T(cone); p.drawEllipse(pt,cr,cr)

        # start/finish
        pen=QPen(C_ORANGE); pen.setWidthF(2.5); pen.setStyle(Qt.DashLine); p.setPen(pen)
        p.drawLine(T(c['outer'][0]),T(c['inner'][0]))
        p.setBrush(QBrush(C_ORANGE)); p.setPen(Qt.NoPen)
        for bp in [c['outer'][0],c['inner'][0]]: p.drawEllipse(T(bp),6,6)

        # direction arrow
        cl2=c['center']; idx=len(cl2)//8
        if idx+40<len(cl2):
            a=T(cl2[idx]); b=T(cl2[idx+40])
            ang=math.atan2(b.y()-a.y(),b.x()-a.x())
            p.save(); p.translate(a); p.rotate(math.degrees(ang))
            p.setBrush(QBrush(QColor(200,200,200,60))); p.setPen(Qt.NoPen)
            p.drawPolygon([QPointF(20,0),QPointF(-9,6),QPointF(-9,-6)]); p.restore()

        p.end()


#  DETAIL DIALOG
class CircuitDetailDialog(QDialog):
    def __init__(self, circuit, parent=None):
        super().__init__(parent); self.circuit=circuit
        self.setWindowTitle(f"Circuit #{circuit['seed']}")
        self.setMinimumSize(960,600); self._build()

    def _lbl_key(self,txt):
        l=QLabel(txt); l.setStyleSheet("color:#606060; font-size:10px; letter-spacing:1px;"); return l
    def _lbl_val(self,txt,col="#d8d8d8"):
        l=QLabel(txt); l.setStyleSheet(f"color:{col}; font-size:13px; font-weight:bold;"); return l

    def _build(self):
        root=QHBoxLayout(self); root.setSpacing(20); root.setContentsMargins(16,16,16,16)

        self.canvas=DetailCanvas(self.circuit,self); root.addWidget(self.canvas,3)

        panel=QWidget(); pl=QVBoxLayout(panel); pl.setSpacing(14); root.addWidget(panel,2)

        ok=self.circuit['compliant']
        lbl_id=QLabel(f"#{self.circuit['seed']}")
        lbl_id.setStyleSheet("font-size:20px; font-weight:bold; color:#d8d8d8; letter-spacing:2px;")
        lbl_ok=QLabel("FSG 2026  COMPLIANT" if ok else "NON-COMPLIANT")
        lbl_ok.setStyleSheet(f"font-size:11px; letter-spacing:3px; color:{'#64af64' if ok else '#bf5050'};")
        pl.addWidget(lbl_id); pl.addWidget(lbl_ok)
        pl.addWidget(self._sep())

        # Rules
        gb=QGroupBox("RULE CHECK"); gl=QGridLayout(gb); gl.setSpacing(4)
        for row,(key,r) in enumerate(self.circuit['rules'].items()):
            bg='#1a221a' if r['pass'] else '#221a1a'
            w=QWidget(); w.setStyleSheet(f"background:{bg}; border:1px solid {'#2a3a2a' if r['pass'] else '#3a2a2a'};")
            wl=QHBoxLayout(w); wl.setContentsMargins(8,5,8,5)
            lk=QLabel(r['label']); lk.setStyleSheet("color:#606060; font-size:10px; min-width:130px;")
            lv=QLabel(r['fmt']);   lv.setStyleSheet(f"color:{'#70b070' if r['pass'] else '#c06060'}; font-size:12px; font-weight:bold;")
            lr=QLabel(r['req']);   lr.setStyleSheet("color:#404040; font-size:9px;"); lr.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
            wl.addWidget(lk); wl.addWidget(lv,1); wl.addWidget(lr)
            gl.addWidget(w,row,0)
        pl.addWidget(gb)

        # Parámetros
        gb2=QGroupBox("PARAMETERS"); pg=QGridLayout(gb2); pg.setSpacing(6)
        c=self.circuit
        stats=[("Length",f"{c['params']['length']:.1f} m"),
               ("Width",  f"{c['params']['width']:.2f} m"),
               ("Cone spacing",f"{c['params']['spacing']:.2f} m"),
               ("Azules",   str(c['params']['n_blue'])),
               ("Amarillos",str(c['params']['n_yellow'])),
               ("Total",    str(c['params']['n_blue']+c['params']['n_yellow']))]
        for i,(k,v) in enumerate(stats):
            lk=QLabel(k); lk.setStyleSheet("color:#555555; font-size:9px; letter-spacing:1px;")
            lv=QLabel(v); lv.setStyleSheet("color:#c8c8c8; font-size:13px; font-weight:bold;")
            pg.addWidget(lk,i,0); pg.addWidget(lv,i,1)
        pl.addWidget(gb2)

        # Export
        gb3=QGroupBox("EXPORT"); el=QHBoxLayout(gb3)
        for txt,fn in [("PCD", lambda: self._save('pcd')),
                       ("JSON",lambda: self._save('json')),
                       ("PCD + JSON",lambda: self._save('both'))]:
            b=QPushButton(txt); b.clicked.connect(fn); el.addWidget(b)
        pl.addWidget(gb3)

        # Leyenda
        leg=QLabel("● Blue = left border (outer)   ● Yellow = right border (inner)\n"
                   "● Orange = start/finish (big orange)   → Clockwise (CW)\n"
                   "Start aligned with ARUSSim origin: (0,0) facing -X")
        leg.setStyleSheet("font-size:8px; color:#404040; letter-spacing:1px;"); leg.setWordWrap(True)
        pl.addWidget(leg); pl.addStretch()

    def _sep(self):
        f=QFrame(); f.setFrameShape(QFrame.HLine); f.setStyleSheet("color:#2a2a2a;"); return f

    def _save(self, fmt):
        seed=self.circuit['seed']; base=f"circuit_{seed}"
        if fmt=='pcd':
            p,_=QFileDialog.getSaveFileName(self,"Save PCD",base+".pcd","PCD (*.pcd)")
            if p: export_pcd(self.circuit,p)
        elif fmt=='json':
            p,_=QFileDialog.getSaveFileName(self,"Save JSON",base+".json","JSON (*.json)")
            if p: export_json(self.circuit,p)
        else:
            p,_=QFileDialog.getSaveFileName(self,"Base name",base,"All (*)")
            if p: export_both(self.circuit,str(p))


#  MAIN WINDOW
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ARUS — Circuit Generator FSG 2026")
        self.setMinimumSize(1200, 740)
        self.setStyleSheet(QSS)
        self._circuits=[]; self._cards=[]; self._thread=None
        self._build(); self.statusBar().showMessage(
            f"Ready  —  modules: points_to_circuit={'OK' if _HAS_PROFILES else 'N/A'}  "
            f"mapFile={'OK' if _HAS_MAPFILE else 'N/A'}"
        )

    # ── Layout ──────────────────────────────────────────────────────────────
    def _build(self):
        root=QWidget(); self.setCentralWidget(root)
        rl=QHBoxLayout(root); rl.setSpacing(0); rl.setContentsMargins(0,0,0,0)
        rl.addWidget(self._build_panel(), 0)
        rl.addWidget(self._build_main(),  1)
        self.setStatusBar(QStatusBar())

    def _build_panel(self):
        panel=QWidget(); panel.setFixedWidth(280)
        panel.setStyleSheet("background:#161616; border-right:1px solid #2a2a2a;")
        ll=QVBoxLayout(panel); ll.setSpacing(12); ll.setContentsMargins(14,14,14,14)

        # Header
        title=QLabel("ARUS\nCIRCUIT GENERATOR")
        title.setStyleSheet("font-size:16px; font-weight:bold; color:#c89030; letter-spacing:4px; line-height:1.5;")
        sub=QLabel("FS-Rules 2026 v1.1 / DS 1")
        sub.setStyleSheet("font-size:9px; color:#404040; letter-spacing:3px;")
        ll.addWidget(title); ll.addWidget(sub)
        ll.addWidget(self._sep())

        # Generation
        gb=self._group("GENERATION"); gl=QVBoxLayout(gb)
        self.sp_count = self._spin("Circuits:", 1, 500, 12, gl)
        self.sp_seed  = self._spin("Base seed:", 0, 999999, 2026, gl)
        self.chk_ok   = QCheckBox("Compliant only (FSG 2026)")
        self.chk_ok.setChecked(True); gl.addWidget(self.chk_ok)
        ll.addWidget(gb)

        # Parámetros (0 = libre)
        gb2=self._group("TRACK (0 = free)"); tl=QVBoxLayout(gb2)
        self.ds_length  = self._dspin("Length (m):",  0, 500, 0, tl)
        self.ds_width   = self._dspin("Width (m):",   0, 8,   0, tl)
        self.ds_spacing = self._dspin("Cone spacing (m):",0, 5,   0, tl)
        self.sp_ctrl    = self._spin( "Ctrl. points:",  0, 20,  0, tl)
        ll.addWidget(gb2)

        # Botón
        self.btn_gen=QPushButton("GENERATE"); self.btn_gen.setObjectName("btn_gen")
        self.btn_gen.clicked.connect(self._on_gen); ll.addWidget(self.btn_gen)

        self.pbar=QProgressBar(); self.pbar.setTextVisible(False); self.pbar.setFixedHeight(6)
        ll.addWidget(self.pbar)

        self.lbl_stats=QLabel("No data.")
        self.lbl_stats.setStyleSheet("font-size:9px; color:#505050; letter-spacing:1px;")
        self.lbl_stats.setWordWrap(True); ll.addWidget(self.lbl_stats)
        ll.addWidget(self._sep())

        # Export all
        gb3=self._group("EXPORT ALL"); el=QVBoxLayout(gb3)
        self.btn_epcd=QPushButton("Export all — PCD")
        self.btn_ejson=QPushButton("Export all — JSON")
        self.btn_eboth=QPushButton("Export all — PCD + JSON")
        for b in [self.btn_epcd,self.btn_ejson,self.btn_eboth]:
            b.setEnabled(False); b.clicked.connect(self._on_export_all); el.addWidget(b)
        ll.addWidget(gb3)

        self.btn_clear=QPushButton("Clear grid")
        self.btn_clear.setEnabled(False); self.btn_clear.clicked.connect(self._clear)
        ll.addWidget(self.btn_clear)
        ll.addStretch()

        leg=QLabel("● Blue = left border\n● Yellow = right border\n"
                   "● Orange = start/finish\n→ CW — start at (0,0) +X")
        leg.setStyleSheet("font-size:8px; color:#383838; letter-spacing:1px;"); ll.addWidget(leg)
        return panel

    def _build_main(self):
        w=QWidget(); wl=QVBoxLayout(w); wl.setSpacing(0); wl.setContentsMargins(0,0,0,0)

        # Barra superior
        bar=QWidget(); bar.setStyleSheet("background:#141414; border-bottom:1px solid #2a2a2a;")
        bl=QHBoxLayout(bar); bl.setContentsMargins(14,8,14,8)
        self.lbl_bar=QLabel("Configure parameters and press  GENERATE")
        self.lbl_bar.setStyleSheet("font-size:10px; color:#505050; letter-spacing:2px;")
        bl.addWidget(self.lbl_bar); bl.addStretch()
        wl.addWidget(bar)

        self.prog_top=QProgressBar(); self.prog_top.setTextVisible(False)
        self.prog_top.setFixedHeight(3)
        self.prog_top.setStyleSheet("QProgressBar{background:#141414;border:none;} QProgressBar::chunk{background:#c89030;}")
        wl.addWidget(self.prog_top)

        self.scroll=QScrollArea(); self.scroll.setWidgetResizable(True)
        self.grid_w=QWidget(); self.grid_w.setStyleSheet("background:#121212;")
        self.grid_l=QGridLayout(self.grid_w)
        self.grid_l.setSpacing(8); self.grid_l.setContentsMargins(12,12,12,12)
        self.grid_l.setAlignment(Qt.AlignTop|Qt.AlignLeft)
        self.scroll.setWidget(self.grid_w); wl.addWidget(self.scroll)
        return w

    #  Widgets helpers 
    def _sep(self):
        f=QFrame(); f.setFrameShape(QFrame.HLine); f.setStyleSheet("color:#282828; background:#282828; max-height:1px;"); return f

    def _group(self, title):
        gb=QGroupBox(title)
        gb.setStyleSheet("""QGroupBox{font-family:"Courier New",monospace;font-size:9px;color:#505050;
            letter-spacing:2px;border:1px solid #282828;margin-top:14px;padding-top:12px;}
            QGroupBox::title{subcontrol-origin:margin;left:8px;padding:0 4px;}""")
        return gb

    def _spin(self,lbl,mn,mx,val,parent_l):
        w=QWidget(); l=QHBoxLayout(w); l.setContentsMargins(0,0,0,0); l.setSpacing(6)
        lb=QLabel(lbl); lb.setStyleSheet("font-size:10px; color:#585858;")
        sp=QSpinBox(); sp.setRange(mn,mx); sp.setValue(val)
        l.addWidget(lb,1); l.addWidget(sp); parent_l.addWidget(w); return sp

    def _dspin(self,lbl,mn,mx,val,parent_l):
        w=QWidget(); l=QHBoxLayout(w); l.setContentsMargins(0,0,0,0); l.setSpacing(6)
        lb=QLabel(lbl); lb.setStyleSheet("font-size:10px; color:#585858;")
        sp=QDoubleSpinBox(); sp.setRange(mn,mx); sp.setValue(val); sp.setDecimals(1)
        l.addWidget(lb,1); l.addWidget(sp); parent_l.addWidget(w); return sp

    #  Logic 
    def _on_gen(self):
        if self._thread and self._thread.isRunning():
            self._thread.stop(); self._reset_btn(); return
        self._clear()
        new_session()   # key: new session seed every time it is pressed

        count=self.sp_count.value(); seed=self.sp_seed.value()
        params={}
        if self.ds_length.value()>0:  params['length']  = self.ds_length.value()
        if self.ds_width.value()>0:   params['width']   = self.ds_width.value()
        if self.ds_spacing.value()>0: params['spacing'] = self.ds_spacing.value()
        if self.sp_ctrl.value()>0:    params['n_ctrl']  = self.sp_ctrl.value()

        self.btn_gen.setText("STOP"); self.btn_gen.setEnabled(True)
        self.pbar.setRange(0,count); self.pbar.setValue(0)
        self.prog_top.setRange(0,count); self.prog_top.setValue(0)
        self.btn_clear.setEnabled(False)
        for b in [self.btn_epcd,self.btn_ejson,self.btn_eboth]: b.setEnabled(False)

        self._thread=GeneratorThread(count,seed,params)
        self._thread.circuit_ready.connect(self._on_ready)
        self._thread.progress.connect(self._on_progress)
        self._thread.finished_all.connect(self._on_done)
        self._thread.start()

    def _on_ready(self, circuit, _):
        if self.chk_ok.isChecked() and not circuit['compliant']: return
        self._circuits.append(circuit)
        card=CircuitCard(circuit); card.clicked.connect(self._show_detail)
        self._cards.append(card)
        n=len(self._cards)-1; nc=self._ncols()
        self.grid_l.addWidget(card, n//nc, n%nc)

    def _on_progress(self, done, total):
        self.pbar.setValue(done); self.prog_top.setValue(done)
        ok=sum(1 for c in self._circuits if c['compliant'])
        self.lbl_stats.setText(f"Processed: {done}/{total}\nShown:  {len(self._circuits)}\nCompliant: {ok}")
        self.lbl_bar.setText(f"Generando...  {done} / {total}")

    def _on_done(self, ok, total):
        self._reset_btn(); self.btn_clear.setEnabled(True)
        has=bool(self._circuits)
        for b in [self.btn_epcd,self.btn_ejson,self.btn_eboth]: b.setEnabled(has)
        rate=ok/total*100 if total else 0
        self.lbl_stats.setText(f"Total: {total}  —  Compliant: {ok}  ({rate:.0f}%)\nShown: {len(self._circuits)}")
        self.lbl_bar.setText(f"{len(self._circuits)} circuits shown  —  {ok}/{total} compliant  ({rate:.0f}%)")
        self.statusBar().showMessage(f"Done — {ok}/{total} FSG 2026 compliant")

    def _reset_btn(self):
        self.btn_gen.setText("GENERATE"); self.btn_gen.setEnabled(True)

    def _show_detail(self, circuit):
        dlg=CircuitDetailDialog(circuit,self); dlg.setStyleSheet(QSS); dlg.exec_()

    def _clear(self):
        for c in self._cards: self.grid_l.removeWidget(c); c.deleteLater()
        self._cards=[]; self._circuits=[]
        self.pbar.setValue(0); self.prog_top.setValue(0)
        self.lbl_stats.setText("No data."); self.lbl_bar.setText("Configure parameters and press  GENERATE")
        self.btn_clear.setEnabled(False)
        for b in [self.btn_epcd,self.btn_ejson,self.btn_eboth]: b.setEnabled(False)

    def _on_export_all(self):
        folder=QFileDialog.getExistingDirectory(self,"Export folder")
        if not folder: return
        txt=self.sender().text()
        fmt='json' if 'JSON' in txt and 'PCD' not in txt else 'pcd' if 'PCD' in txt and 'JSON' not in txt else 'both'
        out=Path(folder)
        for c in self._circuits:
            base=str(out/f"circuit_{c['seed']}")
            if fmt=='pcd': export_pcd(c,base+'.pcd')
            elif fmt=='json': export_json(c,base+'.json')
            else: export_both(c,base)
        QMessageBox.information(self,"Export complete",f"{len(self._circuits)} circuits → {folder}")

    def _ncols(self):
        vp=self.scroll.viewport()
        w=vp.width() if vp else 900
        return max(1,(w-24)//(CircuitCard.W+8))

    def resizeEvent(self,e):
        super().resizeEvent(e)
        if self._cards:
            nc=self._ncols()
            for i,c in enumerate(self._cards): self.grid_l.addWidget(c,i//nc,i%nc)


#  MAIN
if __name__ == '__main__':
    app=QApplication(sys.argv)
    app.setApplicationName("ARUS Circuit Generator")
    win=MainWindow(); win.show()
    sys.exit(app.exec_())
