import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QCoreApplication

class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()


    def initUI(self):
        btn = QPushButton("나가기", self)
        btn.move(110, 300)
        btn.resize(btn.sizeHint())
        btn.clicked.connect(QCoreApplication.instance().quit)

        self.setWindowTitle("Quit Button")
        self.setWindowIcon(QIcon("/home/jun/kang/src/project/project/mart.png"))
        self.setGeometry(700,500,300,350)
        self.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = MyApp()
    sys.exit(app.exec_())