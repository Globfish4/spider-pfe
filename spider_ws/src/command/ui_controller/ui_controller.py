

# GUI
import sys
import numpy as np

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from ui_controller.controller_window import Ui_Form



# ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
import threading
from rclpy.executors import MultiThreadedExecutor






class UiController(QDialog):

    standing = False
    laying = True
    walking = False
    turning = False
    cnt = 0
    on = True


    def __init__(self,parent=None):
        # GUI
        super(UiController, self).__init__(parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start(20) # 20ms

        self.add_motor_jauges()
        self.motors_names = ["revA_11", "revB_11","rev_12","revA_21", "revB_21","rev_22","revA_31", "revB_31", "rev_32", "revA_41", "revB_41", "rev_42"]


        #ROS
        self.cmd = Float64MultiArray()
        # self.cmd.data = [0.0, 1.0, 2.0, 3.0] # Commande en radians
        self.ON = Bool()
        self.ON.data = True

        rclpy.init(args=None)
        self.pub_node = Node('pub_path')
        self.pub_cmd = self.pub_node.create_publisher(Float64MultiArray, '/cmd', 10)
        self.pub_on = self.pub_node.create_publisher(Bool, '/on', 10)
        self.sub_actual_pos = self.pub_node.create_subscription(Float64MultiArray, 'real_position', self.pos_callback, 10)


        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.pub_node)

        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()

        self.pub_node.get_logger().info("UiController node started")
 



    def add_motor_jauges(self):

        self.indicators = {
            "revA_11":None, 
            "revB_11":None,
            "rev_12":None,
            "revA_21":None, 
            "revB_21":None,
            "rev_22":None,
            "revA_31":None, 
            "revB_31":None, 
            "rev_32":None, 
            "revA_41":None, 
            "revB_41":None, 
            "rev_42":None
        }
        joints_limits = {
            "revA_11":(-50, 50),
            "revB_11":(-50, 50),
            "rev_12":(30, 100),
            "revA_21":(-50, 50),
            "revB_21":(-50, 50),
            "rev_22":(30, 100),
            "revA_31":(-50, 50),
            "revB_31":(-50, 50),
            "rev_32":(30, 100),
            "revA_41":(-50, 50),
            "revB_41":(-50, 50),
            "rev_42":(30, 100)
        }
        for k in range(len(self.indicators.keys())):
            key = list(self.indicators.keys())[k]
            self.indicators[key] = MotorIndicator(key)
            low_lim, up_lim = joints_limits[key]
            self.indicators[key].set_angle_limit(low_lim, up_lim)
            col = (k % 4) * 2 + 1
            row = (k % 3) * 2
            self.ui.layout.addWidget(self.indicators[key], row, col, 2, 2)



    def actualize_motors_pos(self):
        for k in range(len(self.motors_names)):
            self.indicators[self.motors_names[k]].set_angle(self.cmd.data[k])


    def pos_callback(self, msg):
        # motors_names = ["revA_11", "revB_11","rev_12","revA_21", "revB_21","rev_22","revA_31", "revB_31", "rev_32", "revA_41", "revB_41", "rev_42"]
        # print(msg.data)
        # self.pub_node.get_logger().info(f"data : {msg.data}")
        for k in range(len(self.motors_names)):
            self.indicators[self.motors_names[k]].get_real_angle(msg.data[k])
            # print()



    def timer_callback(self):

        if self.standing:
            self.cmd.data = stand(self.cnt)
        if self.laying:
            self.cmd.data = lay(self.cnt)
        if self.walking:
            self.cmd.data = walk(self.cnt)
        if self.turning:
            self.cmd.data = turn(self.cnt)

        self.cnt += 1
        self.pub_cmd.publish(self.cmd)
        self.pub_on.publish(self.ON)
        self.actualize_motors_pos()

        if not self.on:
            self.ON.data = False
            if self.cnt > 10:
                self.shutdown_node()



    def stand_up(self):
        print("stand up!")
        self.standing = True
        self.laying = False
        self.walking = False
        self.turning = False
        self.cnt = 0

    def lay_down(self):
        print("lay down!")
        self.standing = False
        self.laying = True
        self.walking = False
        self.turning = False
        self.cnt = 0

    def walk(self):
        print("walk!")
        self.standing = False
        self.laying = False
        self.walking = True
        self.turning = False
        self.cnt = 0

    def turn(self):
        print("turn!")
        self.standing = False
        self.laying = False
        self.walking = False
        self.turning = True
        self.cnt = 0

    def quit(self):
        self.on = False
        self.cnt = 0






    def shutdown_node(self):
        self.pub_node.get_logger().info("Shutting down UiController")
        # self.ros_thread.
        self.timer.stop()
        rclpy.shutdown()
        self.close()
        self.quit()












class MotorIndicator(QWidget):
    def __init__(self, name, parent=None):
        super().__init__(parent)
        self.setMinimumSize(100, 100)
        self.angle_rad_red = 0.0  # angle voulu du moteur (en radians)
        self.angle_rad_green = 0.0  # angle actuel du moteur (en radians)
        self.angle_min = -90  # borne inférieure (en degrés)
        self.angle_max = 90  # borne supérieure
        self.name = name
        self.setObjectName("name")
    
    def set_angle_limit(self, angle_min, angle_max):
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.update()

    def set_angle(self, angle):
        self.angle_rad_red = max(self.angle_min/180*np.pi, min(self.angle_max/180*np.pi, angle))
        # self.update()  # Redessine le widget
    
    def get_real_angle(self, angle):
        self.angle_rad_green = max(self.angle_min/180*np.pi, min(self.angle_max/180*np.pi, angle))
        self.update()  # Redessine le widget

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Définir les dimensions
        rect = self.rect()
        center = rect.center()
        radius = min(rect.width(), rect.height()) // 2 - 10

        # Dessiner disque noir
        painter.setBrush(QColor(0, 0, 0))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(center, radius, radius)

        # Dessiner l’anneau gris partiel (arc de cercle)
        arc_rect = QRect(center.x() - radius + 15, center.y() - radius + 15, 2 * radius - 30, 2 * radius - 30)
        start_angle = (90 - self.angle_max) * 16  # en 1/16 de degré
        span_angle = (self.angle_max - self.angle_min) * 16

        pen = QPen(QColor(180, 180, 180), 10)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawArc(arc_rect, start_angle, span_angle)

        # Dessiner l’aiguille rouge
        painter.setPen(QPen(Qt.red, 2))
        x = center.x() + radius * np.cos(np.pi/2 - self.angle_rad_red)
        y = center.y() - radius * np.sin(np.pi/2 - self.angle_rad_red)
        painter.drawLine(center, QPoint(int(x), int(y)))

        # Dessiner l’aiguille verte
        pen = QPen()
        pen.setColor(QColor(0, 255, 0, 128))  # Vert avec alpha 128 (sur 255)
        pen.setWidth(2)
        painter.setPen(pen)
        x = center.x() + radius * np.cos(np.pi/2 - self.angle_rad_green)
        y = center.y() - radius * np.sin(np.pi/2 - self.angle_rad_green)
        painter.drawLine(center, QPoint(int(x), int(y)))
















def stand(cnt):
    if cnt < 25:
        action = [0.0, np.pi/4, 3*np.pi/8, 0.0, np.pi/4, 3*np.pi/8, 0.0, np.pi/4, 3*np.pi/8, 0.0, np.pi/4, 3*np.pi/8]
    else:
        action = [0.0, -np.pi/8, 3*np.pi/8, 0.0, -np.pi/8, 3*np.pi/8, 0.0, -np.pi/8, 3*np.pi/8, 0.0, -np.pi/8, 3*np.pi/8]
    return action



def lay(cnt):
    if cnt < 25:
        action = [0.0, np.pi/4, 3*np.pi/4, 0.0, np.pi/4, 3*np.pi/4, 0.0, np.pi/4, 3*np.pi/4, 0.0, np.pi/4, 3*np.pi/4]
    else:
        action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    return action




def walk(cnt):

    # Take neutral stable pose
    if cnt < 30:
        action = [
            0.0, 2*np.pi/12, 8*np.pi/12, 
            np.pi/12, 2*np.pi/12, 8*np.pi/12, 
            0.0, 2*np.pi/12, 8*np.pi/12, 
            -np.pi/12, 2*np.pi/12, 8*np.pi/12
        ]
        return action

    # Raise two opposite legs and rotate them in the same direction
    if (cnt - 30) % 120 <= 30:
        action = [
            2*np.pi/12, 4*np.pi/12, 8*np.pi/12, 
            2*np.pi/12, 0.0, 8*np.pi/12, 
            -2*np.pi/12, 4*np.pi/12, 8*np.pi/12, 
            -2*np.pi/12, 0.0, 8*np.pi/12
        ]
        return action

    # Put those two legs down
    if (cnt - 30) % 120 <= 60:
        action = [
            2*np.pi/12, 2*np.pi/12, 8*np.pi/12,
            2*np.pi/12, 0.0, 8*np.pi/12,
            -2*np.pi/12, 2*np.pi/12, 8*np.pi/12,
            -2*np.pi/12, 0.0, 8*np.pi/12
        ]
        return action
    
    # Rotate body while raising the two other opposite legs
    if (cnt - 30) % 120 <= 90:
        action = [
            -2*np.pi/12, 0.0, 8*np.pi/12,
            -2*np.pi/12, 4*np.pi/12, 8*np.pi/12,
            2*np.pi/12, 0.0, 8*np.pi/12, 
            2*np.pi/12, 4*np.pi/12, 8*np.pi/12
        ]
        return action
    
    # put down the two legs
    if (cnt - 30) % 120 <= 120:
        action = [
            -2*np.pi/12, 0.0, 8*np.pi/12,
            -2*np.pi/12, 2*np.pi/12, 8*np.pi/12,
            2*np.pi/12, 0.0, 8*np.pi/12,
            2*np.pi/12, 2*np.pi/12, 8*np.pi/12
        ]
        return action










def turn(cnt):

    # Take neutral stable pose
    if cnt < 30:
        action = [
            0.0, 2*np.pi/12, 8*np.pi/12, 
            np.pi/12, 2*np.pi/12, 8*np.pi/12, 
            0.0, 2*np.pi/12, 8*np.pi/12, 
            -np.pi/12, 2*np.pi/12, 8*np.pi/12
        ]
        return action

    # Raise two opposite legs and rotate them in the same direction
    if (cnt - 30) % 120 <= 30:
        action = [
            2*np.pi/12, 4*np.pi/12, 8*np.pi/12, 
            -2*np.pi/12, 0.0, 8*np.pi/12, 
            2*np.pi/12, 4*np.pi/12, 8*np.pi/12, 
            -2*np.pi/12, 0.0, 8*np.pi/12
        ]
        return action

    # Put those two legs down
    if (cnt - 30) % 120 <= 60:
        action = [
            2*np.pi/12, 2*np.pi/12, 8*np.pi/12,
            -2*np.pi/12, 0.0, 8*np.pi/12,
            2*np.pi/12, 2*np.pi/12, 8*np.pi/12,
            -2*np.pi/12, 0.0, 8*np.pi/12
        ]
        return action
    
    # Rotate body while raising the two other opposite legs
    if (cnt - 30) % 120 <= 90:
        action = [
            -2*np.pi/12, 0.0, 8*np.pi/12,
            2*np.pi/12, 4*np.pi/12, 8*np.pi/12,
            -2*np.pi/12, 0.0, 8*np.pi/12, 
            2*np.pi/12, 4*np.pi/12, 8*np.pi/12
        ]
        return action
    
    # put down the two legs
    if (cnt - 30) % 120 <= 120:
        action = [
            -2*np.pi/12, 0.0, 8*np.pi/12,
            2*np.pi/12, 2*np.pi/12, 8*np.pi/12,
            -2*np.pi/12, 0.0, 8*np.pi/12,
            2*np.pi/12, 2*np.pi/12, 8*np.pi/12
        ]
        return action

    



def main():
    app = QApplication(sys.argv)
    window = UiController()
    window.show()
    sys.exit(app.exec_())



if __name__ == '__main__':
    main()
