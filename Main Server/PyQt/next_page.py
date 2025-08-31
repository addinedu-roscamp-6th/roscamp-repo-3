from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QSpinBox, QComboBox,
    QHBoxLayout, QMessageBox, QListWidget, QListWidgetItem, QStackedWidget
)
import sys

class OrderApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("주문 시스템")
        self.resize(400, 400)

        self.order_completed = False  # 주문 완료 여부

        # 스택 위젯으로 페이지 전환
        self.stack = QStackedWidget()
        self.page1 = self.create_order_page()
        self.page2 = self.create_control_page()
        self.stack.addWidget(self.page1)
        self.stack.addWidget(self.page2)

        layout = QVBoxLayout(self)
        layout.addWidget(self.stack)

    def create_order_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        self.id_spin = QSpinBox()
        self.product_combo = QComboBox()
        self.product_combo.addItems(["사과", "바나나", "오렌지"])
        self.quantity_spin = QSpinBox()

        save_btn = QPushButton("저장")
        save_btn.clicked.connect(self.save_order)

        self.order_list = QListWidget()
        self.order_list.itemDoubleClicked.connect(self.modify_or_delete_order)

        order_btn = QPushButton("주문")
        order_btn.clicked.connect(self.check_order)

        layout.addWidget(QLabel("ID"))
        layout.addWidget(self.id_spin)
        
        layout.addWidget(QLabel("물품"))
        layout.addWidget(self.product_combo)
        layout.addWidget(QLabel("수량"))
        layout.addWidget(self.quantity_spin)
        layout.addWidget(save_btn)
        layout.addWidget(QLabel("주문 내역"))
        layout.addWidget(self.order_list)
        layout.addWidget(order_btn)

        return page

    def create_control_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        self.move_btn = QPushButton("이동")
        self.move_btn.setCheckable(True)
        self.stop_btn = QPushButton("정지")
        self.stop_btn.setCheckable(True)
        self.finish_btn = QPushButton("Finish")

        nav_btn_layout = QHBoxLayout()
        back_btn = QPushButton("← 이전")
        forward_btn = QPushButton("→ 다음")

        back_btn.clicked.connect(self.go_back)
        forward_btn.clicked.connect(self.go_forward)

        nav_btn_layout.addWidget(back_btn)
        nav_btn_layout.addWidget(forward_btn)

        self.move_btn.clicked.connect(self.move_pressed)
        self.stop_btn.clicked.connect(self.stop_pressed)
        self.finish_btn.clicked.connect(self.reset_buttons)

        layout.addLayout(nav_btn_layout)
        layout.addWidget(self.move_btn)
        layout.addWidget(self.stop_btn)
        layout.addWidget(self.finish_btn)

        return page

    def save_order(self):
        id_ = self.id_spin.value()
        product = self.product_combo.currentText()
        quantity = self.quantity_spin.value()
        item_text = f"{id_} - {product} - {quantity}"
        self.order_list.addItem(item_text)

    def modify_or_delete_order(self, item):
        reply = QMessageBox.question(self, "수정 또는 삭제",
                                     "이 항목을 삭제하시겠습니까?",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            row = self.order_list.row(item)
            self.order_list.takeItem(row)

    def check_order(self):
        if self.order_list.count() == 0:
            QMessageBox.warning(self, "경고", "주문 내역이 없습니다.")
            return

        orders = []
        for i in range(self.order_list.count()):
            item_text = self.order_list.item(i).text()
            id_, product, quantity = item_text.split(" - ")
            orders.append({
                "id": int(id_),
                "product": product,
                "quantity": int(quantity)
            })

        confirm = QMessageBox.question(self, "최종 확인",
                                       f"총 {len(orders)}건의 주문을 전송하시겠습니까?",
                                       QMessageBox.Yes | QMessageBox.No)
        if confirm == QMessageBox.Yes:
            self.send_orders(orders)
            self.order_completed = True
            self.stack.setCurrentIndex(1)  # 다음 페이지로 이동

    def send_orders(self, orders):
        import socket, json
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(("상대방_IP", 9999))  # 실제 서버 IP로 변경
            s.sendall(json.dumps(orders).encode('utf-8'))
            s.close()
        except Exception as e:
            QMessageBox.critical(self, "전송 실패", str(e))

    def go_back(self):
        if self.order_completed:
            self.stack.setCurrentIndex(0)

    def go_forward(self):
        if self.order_completed:
            self.stack.setCurrentIndex(1)

    def move_pressed(self):
        self.move_btn.setChecked(True)
        self.stop_btn.setChecked(False)

    def stop_pressed(self):
        self.stop_btn.setChecked(True)
        self.move_btn.setChecked(False)

    def reset_buttons(self):
        self.move_btn.setChecked(False)
        self.stop_btn.setChecked(False)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = OrderApp()
    win.show()
    sys.exit(app.exec_())
