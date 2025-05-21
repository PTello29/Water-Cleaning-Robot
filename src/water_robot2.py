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
        self.set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        self.energy_used = []

    def update_pose(self, data):
        self.pose = data

    def reset_robot(self):
        self.set_pen(0, 0, 0, 0, 1)
        self.teleport(1.0, 1.0, 0.0)
        self.set_pen(255, 255, 255, 3, 0)
        rospy.sleep(1)

    def spawn_trash(self):
        x = random.uniform(2.0, 10.0)
        y = random.uniform(2.0, 10.0)
        self.spawn(x, y, 0, 'trash')
        print(f"Trash spawned at position: x={x:.2f}, y={y:.2f}")  # Nuevo mensaje
        return (x, y)

    def move_to(self, x_goal, y_goal):
        vel = Twist()
        rate = rospy.Rate(10)
        k_linear = 0.5
        k_angular = 3.0
        start_time = time.perf_counter()
        energy = 0.0

        while not rospy.is_shutdown():
            dx = x_goal - self.pose.x
            dy = y_goal - self.pose.y
            dist = sqrt(dx**2 + dy**2)

            angle_to_goal = atan2(dy, dx)
            angle_error = angle_to_goal - self.pose.theta
            angle_error = (angle_error + pi) % (2 * pi) - pi  # Normalize

            if dist < 0.1:
                break

            vel.linear.x = k_linear if abs(angle_error) < 0.1 else 0.0
            vel.angular.z = k_angular * angle_error
            self.vel_pub.publish(vel)

            # Approximate energy = v² + w²
            energy += pow(vel.linear.x, 2) + pow(vel.angular.z, 2)
            rate.sleep()

        vel.linear.x = 0
        vel.angular.z = 0
        self.vel_pub.publish(vel)
        end_time = time.perf_counter()

        total_energy = energy * (end_time - start_time) / 10
        self.energy_used.append(total_energy)
        self.kill("trash")

    def move_in_pattern(self):
        vel = Twist()
        rate = rospy.Rate(10)
        step_size = 1.0  # Tamaño del paso en cada dirección
        directions = [(0, step_size), (step_size, 0), (0, -step_size), (-step_size, 0)]  # Arriba, derecha, abajo, izquierda
        energy = 0.0
        start_time = time.perf_counter()

        for dx, dy in directions:
            target_x = self.pose.x + dx
            target_y = self.pose.y + dy

            # Calcular el ángulo hacia el objetivo
            angle_to_target = atan2(dy, dx)

            # Girar hacia el ángulo deseado
            while not rospy.is_shutdown():
                angle_error = angle_to_target - self.pose.theta
                angle_error = (angle_error + pi) % (2 * pi) - pi  # Normalizar el ángulo

                if abs(angle_error) < 0.1:  # Si el error angular es pequeño, detener la rotación
                    break

                vel.linear.x = 0.0
                vel.angular.z = 2.0 * angle_error  # Control proporcional para girar
                self.vel_pub.publish(vel)
                rate.sleep()

            # Moverse hacia el objetivo
            while not rospy.is_shutdown():
                dist = sqrt((target_x - self.pose.x)**2 + (target_y - self.pose.y)**2)
                if dist < 0.1:  # Si está cerca del objetivo, detenerse
                    break

                vel.linear.x = 0.5  # Velocidad constante
                vel.angular.z = 0.0  # Sin rotación
                self.vel_pub.publish(vel)

                # Aproximar energía = v²
                energy += pow(vel.linear.x, 2)
                rate.sleep()

        # Detener la tortuga al final del patrón
        vel.linear.x = 0
        vel.angular.z = 0
        self.vel_pub.publish(vel)
        end_time = time.perf_counter()

        total_energy = energy * (end_time - start_time) / 10
        self.energy_used.append(total_energy)
        self.kill("trash")

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