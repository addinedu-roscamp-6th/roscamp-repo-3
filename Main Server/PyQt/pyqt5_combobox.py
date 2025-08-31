import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QComboBox, QSpinBox, QVBoxLayout


class MyApp(QWidget):

    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.lbl = QLabel('', self)
        self.lbl.move(80, 150)

        cb = QComboBox(self)
        cb.addItem('과자')
        cb.addItem('과일')
        cb.addItem('의류')
        cb.addItem('음료')
        cb.move(80, 100)
        cb.activated[str].connect(self.onActivated)

        self.lbl1 = QLabel('Count')
        self.spinbox = QSpinBox()
        self.spinbox.setMinimum(-10)
        self.spinbox.setMaximum(30)
        self.spinbox.setSingleStep(2)
        self.lbl2 = QLabel('0')
        self.spinbox.valueChanged.connect(self.value_changed)
        vbox = QVBoxLayout()
        vbox.addWidget(self.lbl1)
        vbox.addWidget(self.spinbox)
        vbox.addWidget(self.lbl2)
        vbox.addStretch()
        self.setLayout(vbox)


        self.setWindowTitle('User GUI')
        self.setGeometry(100, 100, 300, 200)
        self.show()

    def onActivated(self, text):
        self.lbl.setText(text)
        self.lbl.adjustSize()

    def value_changed(self):
        self.lbl2.setText(str(self.spinbox.value()))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MyApp()
    sys.exit(app.exec_())
