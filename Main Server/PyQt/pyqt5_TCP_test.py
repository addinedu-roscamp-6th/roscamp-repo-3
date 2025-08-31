import sys
import socket
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel

class TCPClient(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt TCP Client")
        self.setGeometry(100, 100, 300, 150)

        self.label = QLabel("서버에 연결되지 않음")
        self.button = QPushButton("서버에 메시지 전송")
        self.button.clicked.connect(self.send_message)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.button)
        self.setLayout(layout)

        # TCP 소켓 생성 및 서버 연결
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client_socket.connect(('192.168.1.8', 9999))  # IP와 포트에 맞게 설정
            self.label.setText("서버에 연결됨")
        except Exception as e:
            self.label.setText(f"연결 실패: {e}")
            self.button.setEnabled(False)

    def send_message(self):
        try:
            message = "Hello from PyQt!"
            self.client_socket.sendall(message.encode())
            self.label.setText("메시지 전송 완료")
        except Exception as e:
            self.label.setText(f"전송 실패: {e}")

    def closeEvent(self, event):
        self.client_socket.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    client = TCPClient()
    client.show()
    sys.exit(app.exec_())