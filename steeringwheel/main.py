#main.py
import sys
from PyQt5.QtWidgets import QApplication
from control.hub import ControlHub
from gui.main_window import ControlHubGUI

def main():
    hub = ControlHub()
    app = QApplication(sys.argv)
    window = ControlHubGUI(hub)
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
