import sys

from robocontrol_bootstrap import ensure_py312_runtime_for_main, configure_qt_runtime_environment


ensure_py312_runtime_for_main(__file__)
configure_qt_runtime_environment()

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt, QCoreApplication

from robocontrol_gui import RoboArmController


if __name__ == '__main__':
    QCoreApplication.setAttribute(Qt.AA_UseSoftwareOpenGL, True)
    app = QApplication(sys.argv)
    ex = RoboArmController()
    ex.show()
    sys.exit(app.exec_())
