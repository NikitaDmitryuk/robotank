#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import subprocess

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        # Инициализация PWM для моторов (примерные пины)
        self.left_motor_forward = PWMControl(4)  # Левый мотор вперёд
        self.left_motor_reverse = PWMControl(3)  # Левый мотор назад
        self.right_motor_forward = PWMControl(21) # Правый мотор вперёд
        self.right_motor_reverse = PWMControl(22) # Правый мотор назад

        # Подписка на данные джойстика
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):
        # Получение данных с джойстика
        linear = msg.axes[1]  # Вперёд/назад (ось Y)
        angular = msg.axes[0] # Поворот (ось X)

        # Расчёт скоростей моторов
        left_speed = linear - angular
        right_speed = linear + angular

        # Ограничение скоростей в диапазоне [-1, 1]
        left_speed = max(min(left_speed, 1.0), -1.0)
        right_speed = max(min(right_speed, 1.0), -1.0)

        print(f"left_speed: {left_speed}, right_speed: {right_speed}")

        # Управление моторами
        self.set_motor_speed(self.left_motor_forward, self.left_motor_reverse, left_speed)
        self.set_motor_speed(self.right_motor_forward, self.right_motor_reverse, right_speed)

    def set_motor_speed(self, forward_pwm, reverse_pwm, speed):
        """Управление направлением и скоростью мотора"""
        if speed > 0:
            forward_pwm.set_duty(abs(speed))  # Вперёд
            reverse_pwm.set_duty(0.0)
        elif speed < 0:
            forward_pwm.set_duty(0.0)
            reverse_pwm.set_duty(abs(speed))  # Назад
        else:
            forward_pwm.set_duty(0.0)         # Остановка
            reverse_pwm.set_duty(0.0)

class PWMControl:
    def __init__(self, wpi_pin):
        self.wpi_pin = wpi_pin
        self.arr = 2048  # Диапазон PWM
        self._setup()

    def _setup(self):
        """Настройка PWM"""
        subprocess.run(['gpio', 'mode', str(self.wpi_pin), 'pwm'])
        subprocess.run(['gpio', 'pwmTone', str(self.wpi_pin), '20000'])  # Частота 20 кГц
        subprocess.run(['gpio', 'pwmr', str(self.wpi_pin), str(self.arr)])
        subprocess.run(['gpio', 'pwmc', str(self.wpi_pin), '1'])

    def set_duty(self, duty):
        """Установка коэффициента заполнения (0.0–1.0)"""
        duty = max(0.0, min(1.0, duty))
        ccr_value = int(duty * self.arr)
        subprocess.run(['gpio', 'pwm', str(self.wpi_pin), str(ccr_value)])

def main(args=None):
    rclpy.init(args=args)
    motor_driver = MotorDriver()
    rclpy.spin(motor_driver)
    motor_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
