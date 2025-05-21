#!/usr/bin/env python3
import rospy, time, random
from math import atan2, sqrt, pow, pi
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class EnergyRobot:
    def __init__(self):
        rospy.init_node("energy_cleaner")
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.spawn = rospy.ServiceProxy('/spawn', Spawn)
        self.kill = rospy.ServiceProxy('/kill', Kill)
        self.teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        self.energy_used = []

    def update_pose(self, data):
        self.pose = data

    def reset_robot(self):
        self.teleport(1.0, 1.0, 0.0)
        rospy.sleep(1)

    def spawn_trash(self):
        x = random.uniform(2.0, 10.0)
        y = random.uniform(2.0, 10.0)
        self.spawn(x, y, 0, 'trash')
        print(f"Trash spawned at position: x={x:.2f}, y={y:.2f}")
        return (x, y)

    def move_in_pattern(self):
        vel = Twist()
        rate = rospy.Rate(10)
        energy = 0.0
        start_time = time.perf_counter()

        # Definir los límites del mapa
        x_min, x_max = 1.0, 10.0
        y_min, y_max = 1.0, 10.0
        step_size = 1.0  # Tamaño del paso en cada dirección

        # Comenzar desde la esquina inferior izquierda
        current_x, current_y = x_min, y_min
        moving_right = True  # Dirección inicial: derecha

        while current_y <= y_max:
            # Mover en la dirección actual (derecha o izquierda)
            target_x = x_max if moving_right else x_min

            # Girar hacia el objetivo en X
            angle_to_target = 0 if moving_right else pi
            while not rospy.is_shutdown():
                angle_error = angle_to_target - self.pose.theta
                angle_error = (angle_error + pi) % (2 * pi) - pi  # Normalizar el ángulo
                if abs(angle_error) < 0.1:  # Si el error angular es pequeño, detener la rotación
                    break
                vel.linear.x = 0.0
                vel.angular.z = 2.0 * angle_error  # Control proporcional para girar
                self.vel_pub.publish(vel)
                rate.sleep()

            # Mover en X
            while not rospy.is_shutdown():
                dist = abs(target_x - self.pose.x)
                if dist < 0.1:  # Si está cerca del objetivo en X, detenerse
                    break
                vel.linear.x = 0.5
                vel.angular.z = 0.0
                self.vel_pub.publish(vel)
                energy += pow(vel.linear.x, 2)
                rate.sleep()

            # Detener el movimiento en X
            vel.linear.x = 0
            self.vel_pub.publish(vel)

            # Subir una unidad en Y
            current_y += step_size
            if current_y > y_max:  # Si se sale del límite superior, detenerse
                break

            # Girar hacia el objetivo en Y
            angle_to_target = pi / 2  # Apuntar hacia arriba
            while not rospy.is_shutdown():
                angle_error = angle_to_target - self.pose.theta
                angle_error = (angle_error + pi) % (2 * pi) - pi  # Normalizar el ángulo
                if abs(angle_error) < 0.1:  # Si el error angular es pequeño, detener la rotación
                    break
                vel.linear.x = 0.0
                vel.angular.z = 2.0 * angle_error  # Control proporcional para girar
                self.vel_pub.publish(vel)
                rate.sleep()

            # Mover en Y
            while not rospy.is_shutdown():
                dist = abs(current_y - self.pose.y)
                if dist < 0.1:  # Si está cerca del objetivo en Y, detenerse
                    break
                vel.linear.x = 0.5
                vel.angular.z = 0.0
                self.vel_pub.publish(vel)
                energy += pow(vel.linear.x, 2)
                rate.sleep()

            # Cambiar la dirección horizontal
            moving_right = not moving_right

        # Detener la tortuga al final del recorrido
        vel.linear.x = 0
        vel.angular.z = 0
        self.vel_pub.publish(vel)
        end_time = time.perf_counter()

        total_energy = energy * (end_time - start_time) / 10
        self.energy_used.append(total_energy)

    def run(self, trials=3):
        for _ in range(trials):
            self.reset_robot()
            self.spawn_trash()
            rospy.sleep(1)
            self.move_in_pattern()

        print("Energy used per attempt:", [f"{e:.2f}" for e in self.energy_used])
        avg = sum(self.energy_used) / len(self.energy_used)
        print(f"Average energy: {avg:.2f}")

if __name__ == "__main__":
    try:
        robot = EnergyRobot()
        rospy.sleep(2)
        robot.run()
    except rospy.ROSInterruptException:
        pass