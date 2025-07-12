import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import OPi.GPIO as GPIO

# --- Настройки ---
# Пины для управления двигателем 1 (левый)
AIN1 = 7  # Вперед (PWM)
AIN2 = 11 # Назад (PWM)

# Пины для управления двигателем 2 (правый)
BIN1 = 13 # Вперед (PWM)
BIN2 = 15 # Назад (PWM)

# Оси джойстика для управления
# `axes[1]` - левый стик вверх/вниз
# `axes[4]` - правый стик вверх/вниз (уточните для вашего джойстика)
LEFT_MOTOR_AXIS = 1
RIGHT_MOTOR_AXIS = 4

DEADZONE = 0.1 # Порог чувствительности стиков

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.get_logger().info('Узел управления двигателями запущен.')

        # Подписка на топик джойстика
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        # Настройка GPIO
        GPIO.setmode(GPIO.BOARD)
        # Настраиваем все пины как выходы
        for pin in [AIN1, AIN2, BIN1, BIN2]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        # Настройка PWM (ШИМ) для каждого направления каждого мотора
        # Частота 1000 Гц - хороший старт для BLDC
        self.pwm_a1 = GPIO.PWM(AIN1, 1000)
        self.pwm_a2 = GPIO.PWM(AIN2, 1000)
        self.pwm_b1 = GPIO.PWM(BIN1, 1000)
        self.pwm_b2 = GPIO.PWM(BIN2, 1000)

        # Запускаем все PWM с нулевой мощностью (duty cycle = 0)
        self.pwm_a1.start(0)
        self.pwm_a2.start(0)
        self.pwm_b1.start(0)
        self.pwm_b2.start(0)

        self.get_logger().info('GPIO и PWM инициализированы.')

    def joy_callback(self, msg):
        # Получаем значения осей
        left_stick_y = msg.axes[LEFT_MOTOR_AXIS]
        right_stick_y = msg.axes[RIGHT_MOTOR_AXIS]

        # Управление левым двигателем
        self.control_motor(self.pwm_a1, self.pwm_a2, left_stick_y)

        # Управление правым двигателем
        self.control_motor(self.pwm_b1, self.pwm_b2, right_stick_y)

    def control_motor(self, pwm_fwd, pwm_rev, speed):
        # Преобразуем скорость (-1.0 до 1.0) в мощность ШИМ (0 до 100)
        duty_cycle = abs(speed * 100)

        if speed > DEADZONE: # Движение вперед
            pwm_rev.ChangeDutyCycle(0)
            pwm_fwd.ChangeDutyCycle(duty_cycle)
        elif speed < -DEADZONE: # Движение назад
            pwm_fwd.ChangeDutyCycle(0)
            pwm_rev.ChangeDutyCycle(duty_cycle)
        else: # Остановка (в мертвой зоне)
            pwm_fwd.ChangeDutyCycle(0)
            pwm_rev.ChangeDutyCycle(0)

    def cleanup(self):
        self.get_logger().info('Остановка двигателей и очистка GPIO.')
        self.pwm_a1.stop()
        self.pwm_a2.stop()
        self.pwm_b1.stop()
        self.pwm_b2.stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorControllerNode()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        motor_controller.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
