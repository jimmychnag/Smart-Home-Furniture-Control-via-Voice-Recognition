import sys
from datetime import datetime
import serial
import serial.tools.list_ports

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtWidgets import (
    QApplication, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QPushButton, QStackedWidget, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget, QComboBox, QSpinBox, QMessageBox
)
from PySide6.QtGui import QFont


class DeviceIndicator(QWidget):
    def __init__(self, name: str, parent: QWidget | None = None):
        super().__init__(parent)
        self._state = False
        self._name = name

        self._led = QLabel()
        self._led.setFixedSize(40, 40)
        self._led.setStyleSheet(self._style_for_state(self._state))
        self._label = QLabel(name)
        font = QFont()
        font.setPointSize(14)
        self._label.setFont(font)

        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addWidget(self._led)
        lay.addWidget(self._label)
        lay.addStretch()

    @staticmethod
    def _style_for_state(state: bool) -> str:
        color = "#4CAF50" if state else "#9E9E9E"
        return f"background: {color}; border-radius: 15px; border: 1px solid #333;"

    def state(self) -> bool:
        return self._state

    def set_state(self, state: bool):
        if state == self._state:
            return
        self._state = state
        self._led.setStyleSheet(self._style_for_state(self._state))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Smart Home")
        self.resize(800, 400)

        self._serial = None
        self._serial_timer = QTimer(self)
        self._serial_timer.timeout.connect(self._poll_serial_data)

        self._uart_state = "WAIT_FOR_HEADER"
        self._uart_pending_state = None
        self._uart_rx_buffer = []

        self._stack = QStackedWidget()
        self.setCentralWidget(self._stack)

        self._page_b = self._create_page_b()
        self._stack.addWidget(self._page_b)

    def _create_page_b(self) -> QWidget:
        page = QWidget()
        vbox = QVBoxLayout(page)
        vbox.setContentsMargins(12, 12, 12, 12)
        vbox.setSpacing(12)

        top_widget = QWidget()
        hbox = QHBoxLayout(top_widget)
        hbox.setContentsMargins(0, 0, 0, 0)
        hbox.setSpacing(12)

        # 裝置指示器
        self._light_indicator = DeviceIndicator("Light")
        self._fan_indicator = DeviceIndicator("Fan")
        self._ac_indicator = DeviceIndicator("AC")
        self._indicators = {
            "Light": self._light_indicator,
            "Fan": self._fan_indicator,
            "AC": self._ac_indicator,
        }

        hbox.addWidget(self._light_indicator)
        hbox.addWidget(self._fan_indicator)
        hbox.addWidget(self._ac_indicator)

        # COM 埠選擇
        #self._com_selector = QComboBox()
        #ports = serial.tools.list_ports.comports()
        #for port in ports:
        #    self._com_selector.addItem(port.device)

        #self._baud_selector = QSpinBox()
        #self._baud_selector.setRange(1200, 460800)
        #self._baud_selector.setValue(460800)

        self._connect_btn = QPushButton("連線")
        self._connect_btn.clicked.connect(self._toggle_serial_connection)

        #hbox.addWidget(QLabel("COM"))
        #hbox.addWidget(self._com_selector)
        #hbox.addWidget(QLabel("Baud"))
        #hbox.addWidget(self._baud_selector)
        hbox.addWidget(self._connect_btn)

        # 手動輸入區
        self._input = QLineEdit()
        self._input.setPlaceholderText("輸入2個 byte，例如：1 4")
        self._input.returnPressed.connect(self._on_input_entered)
        hbox.addWidget(self._input)

        # 紀錄表格
        self._log_table = QTableWidget(0, 3)
        self._log_table.setHorizontalHeaderLabels(["Device", "On/Off", "Time"])
        self._log_table.horizontalHeader().setStretchLastSection(True)
        self._log_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._log_table.setAlternatingRowColors(True)
        self._log_table.setFixedHeight(200)
        self._log_table.setColumnWidth(2, 100)

        vbox.addWidget(top_widget)
        vbox.addWidget(self._log_table, 1)
        return page

    @Slot()
    def _toggle_serial_connection(self):
        if self._serial and self._serial.is_open:
            self._serial_timer.stop()
            self._serial.close()
            self._serial = None
            self._connect_btn.setText("連線")
            QMessageBox.information(self, "Serial", "已關閉串口連線")
        else:
            try:
                #port = self._com_selector.currentText()
                port = "COM8"
                #baud = self._baud_selector.value()
                baud = 460800
                self._serial = serial.Serial(port, baudrate=baud, timeout=0.1)
                self._serial_timer.start(100)
                self._connect_btn.setText("中斷")
                QMessageBox.information(self, "Serial", f"已連接至 {port} @ {baud} bps")
            except Exception as e:
                QMessageBox.critical(self, "錯誤", f"串口開啟失敗: {e}")
                self._serial = None

    @Slot()
    def _on_input_entered(self):
        text = self._input.text().strip().split()
        self._input.clear()
        if len(text) == 2:
            try:
                state_val = int(text[0])
                device_val = int(text[1])

                # 1 = ON, 2 = OFF
                self._uart_pending_state = True if state_val == 1 else False if state_val == 2 else None
                if self._uart_pending_state is not None:
                    self._handle_device_byte(device_val)
            except Exception as e:
                print(f"輸入錯誤: {e}")


    def _poll_serial_data(self):
        if self._serial and self._serial.in_waiting >= 2:
            try:
                data = self._serial.read(2)
                if len(data) != 2:
                    return
                state_byte = data[0]
                device_byte = data[1]

                # 判斷 ON/OFF
                if state_byte == 0x01:
                    self._uart_pending_state = True
                elif state_byte == 0x02:
                    self._uart_pending_state = False
                else:
                    self._uart_pending_state = None

                if self._uart_pending_state is not None:
                    self._handle_device_byte(device_byte)

            except Exception as e:
                print(f"串口讀取錯誤: {e}")


    def _handle_device_byte(self, device_code: int):
        if device_code == 0x04:
            self._update_device("Light", self._uart_pending_state)
        elif device_code == 0x08:
            self._update_device("Fan", self._uart_pending_state)
        elif device_code == 0x10:
            self._update_device("AC", self._uart_pending_state)

    def _update_device(self, device_name: str, new_state: bool):
        indicator = self._indicators[device_name]
        if indicator.state() == new_state:
            return
        indicator.set_state(new_state)

        row = self._log_table.rowCount()
        self._log_table.insertRow(row)
        self._log_table.setItem(row, 0, QTableWidgetItem(device_name))
        self._log_table.setItem(row, 1, QTableWidgetItem("ON" if new_state else "OFF"))
        self._log_table.setItem(row, 2, QTableWidgetItem(datetime.now().strftime("%H:%M:%S")))
        self._log_table.scrollToBottom()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
