import sys
import os
import time 
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout, QGraphicsView, QGraphicsScene
from PyQt5.QtGui import QPixmap, QPainter, QFont, QFontDatabase
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg

app = QApplication(sys.argv)

seven_segment_names = ["Tank Weight (lbs)", "Tank Pressure (psi)", "Thrust (N)"]

start_time = time.perf_counter()

# Get screen size
screen_size = app.primaryScreen().size()
SCREEN_WIDTH = screen_size.width()
SCREEN_HEIGHT = screen_size.height()
scrn_widt_adj_ratio = screen_size.width()/1920
scrn_height_adj_ratio = screen_size.height()/1080

class SevenSegment(QLabel):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        id = QFontDatabase.addApplicationFont('DSEG7Modern-BoldItalic.ttf')
        if id < 0: print("Font Import Error")
        families = QFontDatabase.applicationFontFamilies(id)
        #print(families[0])
        self.setFont(QFont('DSEG7 Modern', int(scrn_widt_adj_ratio*100)))
        self.setStyleSheet("color: red;")
        self.setText("00000")
        self.title_label = QLabel(title, parent)
        self.title_label.setFont(QFont('Arial', int(scrn_widt_adj_ratio*20)))
        self.title_label.setStyleSheet("color: white;")

class MovablePlotWidget(QWidget):
    def __init__(self, plot_widget: pg.PlotWidget, parent=None):
        super().__init__(parent)
        self.pressing = False

        # Embed the provided PlotWidget
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(plot_widget)
        self.setLayout(layout)

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tank Monitor")
        self.setStyleSheet("background-color: black;")
        self.setGeometry(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT)

        # Logo
        logo = QLabel(self)
        logo_pixmap = QPixmap("SLURPL-Logo-Transparent.png")
        logo.setPixmap(logo_pixmap.scaled(int(SCREEN_WIDTH/7), int(SCREEN_HEIGHT/7), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        logo.move(int(6*SCREEN_WIDTH/7)-50, int(6*SCREEN_HEIGHT/7)-50)

        # Valve Display
        self.valves = QLabel(self)
        self.pixmap = QPixmap('./Valve_States/0_Start.png')
        self.valves.setPixmap(self.pixmap.scaled(int(3*SCREEN_WIDTH/5), int(3*SCREEN_HEIGHT/5), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        

        # Graphs
        self.graph_psi = pg.PlotWidget(title="TANK PRESSURE (PSI)")
        self.graph_lbs = pg.PlotWidget(title="TANK WEIGHT (LBS)")
        self.graph_temp = pg.PlotWidget(title="TANK TEMP (C)")
        self.graph_psi.setBackground('k')
        self.graph_lbs.setBackground('k')
        self.graph_temp.setBackground('k')
        for graph in (self.graph_psi, self.graph_lbs ,self.graph_temp):
            graph.showGrid(x=True, y=True)
            graph.setLabel('left', color='white')
            graph.setLabel('bottom', color='white')
            graph.getAxis('left').setTextPen('w')
            graph.getAxis('bottom').setTextPen('w')
        
        psi_movable = MovablePlotWidget(self.graph_psi, self)
        lbs_movable = MovablePlotWidget(self.graph_lbs, self)
        temp_movable = MovablePlotWidget(self.graph_temp, self)
        psi_movable.move(0, int(3*SCREEN_HEIGHT/5))
        lbs_movable.move(int(1*SCREEN_WIDTH/5), int(3*SCREEN_HEIGHT/5))
        temp_movable.move(int(2*SCREEN_WIDTH/5), int(3*SCREEN_HEIGHT/5))
        psi_movable.resize(int(1*SCREEN_WIDTH/5),int(2*SCREEN_HEIGHT/5))
        lbs_movable.resize(int(1*SCREEN_WIDTH/5), int(2*SCREEN_HEIGHT/5))
        temp_movable.resize(int(1*SCREEN_WIDTH/5), int(2*SCREEN_HEIGHT/5))
        

        # Right-side 7-segment display'
        self.lbs_display = SevenSegment("Tank Nitrous Weight (LBS)", self)
        self.psi_display = SevenSegment("Tank Pressure (PSI)", self)
        self.temp_display = SevenSegment("Tank Temperature (C)", self)
        i = 0
        for seven_seg in (self.lbs_display, self.psi_display, self.temp_display):
            seven_seg.move(int(scrn_widt_adj_ratio*1250),i*300+50)
            seven_seg.title_label.move(int(scrn_widt_adj_ratio*1250),i*300)
            i+=1

        # Sample data
        self.x = [0] * 100
        self.y_psi = [0] * 100
        self.y_lbs = [0] * 100
        self.y_temp = [0] * 100
        self.cur_image = 0

        self.psi_line = self.graph_psi.plot(self.x, self.y_psi, pen='g')
        self.lbs_line = self.graph_lbs.plot(self.x, self.y_lbs, pen='b')
        self.temp_line = self.graph_temp.plot(self.x, self.y_temp, pen='y')

        # Timer for updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(100)
        
        self.images = [f for f in os.listdir('./Valve_States/') if f.lower().endswith(('.png', '.jpg', '.jpeg'))]

        self.showFullScreen()

    def update_graphs(self):
        import random
        new_x_time = (time.perf_counter() - start_time)
        new_psi = random.random()*999
        new_lbs = random.random()*20
        new_temp = random.random()*60

        new_img = random.randint(0,5)
        self.change_image(new_img)

        self.x = self.x[1:] + [new_x_time]
        self.y_psi = self.y_psi[1:] + [new_psi]
        self.y_lbs = self.y_lbs[1:] + [new_lbs]
        self.y_temp = self.y_temp[1:] + [new_temp]

        self.psi_display.setText(f"{new_psi:.2f}")
        self.lbs_display.setText(f"{new_lbs:.2f}")
        self.temp_display.setText(f"{new_temp:.2f}")

        self.psi_line.setData(self.x, self.y_psi)
        self.lbs_line.setData(self.x, self.y_lbs)
        self.temp_line.setData(self.x, self.y_temp)

    def change_image(self, index):
        if self.images:
            self.cur_image = index % len(self.images)
            self.pixmap = QPixmap('./Valve_States/'+self.images[self.cur_image])
            self.valves.setPixmap(self.pixmap.scaled(int(3*SCREEN_WIDTH/5), int(3*SCREEN_HEIGHT/5), Qt.KeepAspectRatio, Qt.SmoothTransformation))
            self.valves.repaint()
            
    
window = MainWindow()
#window.change_image(1)
sys.exit(app.exec_())
