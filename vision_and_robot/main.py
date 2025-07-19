import cv2
import cv2.aruco as aruco
from numpy import *
from time import *
from math import *
import socket
import threading
import serial
import serial.tools.list_ports

A, B = 155.0, 400.0
busy = False
current_client = None
last_command_time = 0
INACTIVITY_TIMEOUT = 30  # 30 секунд
counter = 5
last_time = time()
dict_good_markers = {}
dict_bad_markers = {}

class MarkerHandler:
    def __init__(self):
        self.stack_position = (0, 150, 0)  # x, y, z для стопки хороших маркеров
        self.disposal_positions = [
            (300, 150, 0),  # Правая сторона (x > 250)
            (-300, 150, 0)   # Левая сторона (x < -250)
        ]
        self.current_z = 20  # Начальная высота стопки
        self.z_increment = 5  # Прирост высоты для каждого нового маркера
        self.safe_height = 100  # Безопасная высота для перемещений
        self.pick_height = 20   # Высота для подбора маркера
        self.speed = 1500       # Скорость перемещения

    def send_command(self, command: str) -> str:
        """Отправка команды манипулятору и получение ответа"""
        response = send_to_arduino_and_get_response(command)
        time.sleep(0.1)  # Небольшая задержка между командами
        return response

    def setup(self):
        """Настройка робота перед началом работы"""
        self.send_command(f'SET_MAX_SPEED {self.speed} {self.speed} {self.speed}')
        self.send_command('TOOL_VACUUM_OFF')
        return "Настройка завершена"

    def rotate_to_standard(self, current_angle: int) -> int:
        """Выбор стандартного угла поворота (0 или 90 градусов)"""
        return 0 if abs(current_angle) <= 45 else 90

    def move_to(self, x: float, y: float, z: float):
        """Безопасное перемещение в указанные координаты"""
        # Сначала поднимаемся на безопасную высоту
        if z < self.safe_height:
            self.send_command(f'MOVE_TO {x} {y} {self.safe_height}')
        # Затем перемещаемся к целевой точке
        response = self.send_command(f'MOVE_TO {x} {y} {z}')
        return response

    def pick_marker(self, position: tuple) -> str:
        """Подбор маркера с заданной позицией"""
        x, y, angle = position
        target_angle = self.rotate_to_standard(angle)
        
        # Поворот инструмента
        self.send_command(f'TOOL_ROTATE_TO {target_angle}')
        
        # Перемещение к маркеру
        self.move_to(x, y, self.safe_height)
        self.move_to(x, y, self.pick_height)
        
        # Включение вакуума и подъем
        self.send_command('TOOL_VACUUM_ON')
        time.sleep(0.5)  # Даем время на захват
        self.move_to(x, y, self.safe_height)
        
        return f"Маркер в позиции ({x}, {y}) поднят"

    def place_marker(self, target_position: tuple) -> str:
        """Установка маркера в целевую позицию"""
        x, y, _ = target_position
        
        # Перемещение к целевой позиции
        self.move_to(x, y, self.safe_height)
        self.move_to(x, y, self.current_z)
        
        # Выключение вакуума
        self.send_command('TOOL_VACUUM_OFF')
        time.sleep(0.3)  # Даем время на отпускание
        
        # Подъем
        self.move_to(x, y, self.safe_height)
        
        # Увеличение высоты для следующего маркера
        self.current_z += self.z_increment
        
        return f"Маркер установлен в позицию ({x}, {y}, {self.current_z})"

    def dispose_marker(self, position: tuple) -> str:
        """Удаление маркера с рабочего поля"""
        # Выбираем сторону для удаления в зависимости от текущей позиции
        disposal_pos = self.disposal_positions[0] if position[0] < 0 else self.disposal_positions[1]
        
        # Подбираем маркер
        self.pick_marker(position)
        
        # Перемещаем в зону удаления
        result = self.place_marker(disposal_pos)
        
        return f"Маркер удалён с поля. {result}"

    def stack_marker(self, position: tuple) -> str:
        """Добавление маркера в стопку"""
        # Подбираем маркер
        self.pick_marker(position)
        
        # Перемещаем в стопку
        result = self.place_marker(self.stack_position)
        
        return f"Маркер добавлен в стопку. {result}"

    def process_markers(self, dict_good_markers: dict, dict_bad_markers: dict) -> list:
        """Обработка всех маркеров согласно правилам"""
        results = []
        results.append(self.setup())
        
        # Обработка плохих маркеров
        for marker_id, position in dict_bad_markers.items():
            results.append(f"Удаление плохого маркера {marker_id}")
            results.append(self.dispose_marker(position))
        
        # Обработка хороших маркеров (сортировка по убыванию номера)
        good_markers_sorted = sorted(
            ((marker_id, pos) for marker_id, pos in dict_good_markers.items() 
             if marker_id % 10 == 0 and marker_id not in {0, 1, 2, 3}),
            key=lambda x: x[0], 
            reverse=True
        )
        
        for marker_id, position in good_markers_sorted:
            results.append(f"Добавление хорошего маркера {marker_id} в стопку")
            results.append(self.stack_marker(position))
        
        # Возврат в исходное положение
        self.move_to(0, 0, self.safe_height)
        self.send_command('TOOL_ROTATE_TO 0')
        results.append("Обработка маркеров завершена")
        
        return results

def opros():
    return int(input('Введите 0, если пользуетесь вебкамом или 1, если подключили камеру: '))

def center_trapation(vertices):
    # Исходные координаты вершин трапеции (формат: array([[[x1, y1], [x2, y2], [x3, y3], [x4, y4]]])
    points = vertices.reshape(-1, 2)
    center_x = mean(points[:, 0])
    center_y = mean(points[:, 1])
    return [float(center_x), float(center_y)]

def calibration(real_distance, all_good_markers, ids):
    # all_good_markers.sort(key=lambda x: x[1])
    # print(ids[all_good_markers.index(all_good_markers[0])], ids[all_good_markers.index(all_good_markers[-1])])
    # pixel_distance = dist(all_good_markers[0], all_good_markers[-1])
    list_ids = ids.tolist()
    try:
        pixel_distance = dist(all_good_markers[list_ids.index([0])], all_good_markers[list_ids.index([3])])
        rotatio = real_distance / pixel_distance
        a, b = all_good_markers[list_ids.index([0])]
        return rotatio, a*rotatio, b*rotatio  # Пикселей на единицу длины (например, 50 px/см)
    except:
        return 0, 0, 0

def coordinations(centers, ids, rotation_mode, px_ratio, a, b):
    adapted_centers = {}
    for n in range(len(centers)):
        x, y = centers[n]
        adapted_centers[str(ids[n])] = [x*px_ratio - A - a, B + b - y*px_ratio, rotation_mode[n]]
    return adapted_centers

def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        try:
            ser = serial.Serial(port.device, 115200, timeout=1)
            print(port.device)
            sleep(2)  # Ждём, пока Arduino перезагрузится
            ser.flushInput()
            ser.write(b'PING\n')  # Отправляем команду PING
            print(1)
            response = ser.readline().decode().strip()
            print(response)
            if response == 'PONG':
                print(f"Arduino обнаружена на {port.device}")
                return ser
            ser.close()
        except serial.SerialException:
            continue
    print("Arduino не найдена")
    return None

def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def transform_command(command):
    """
    Текущая логика трансформации: для MOVE_TO / MOVE_TO_ROBOT
    меняем координаты, как раньше.
    """
    parts = command.strip().split()
    
    # Пример: "MOVE_TO x y z" или "MOVE_TO_ROBOT x y z"
    if len(parts) == 4 and (parts[0] in ("MOVE_TO", "MOVE_TO_ROBOT")):
        try:
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
        except ValueError:
            return "Invalid coordinate values"

        if parts[0] == "MOVE_TO_ROBOT":
            # Координаты робота напрямую
            x_new = x
            y_new = y
            z_new = z
        else:
            # MOVE_TO: меняем X и Y местами + смещения
            x_new = -y + 552
            y_new = -x + 268
            z_new = z-3
        # Собираем итоговую строку
        return f"{parts[0]} {int(x_new)} {int(y_new)} {int(z_new)}"
    if parts[0] == "TOOL_ROTATE_TO":
        try:
            angle = float(parts[1])
            v = arduino_map(angle, 90, 0, 45, 142)
            return f"{parts[0]} {int(angle)}"
            
        except ValueError:
            return "Invalid angle values"
        
    # Если это не MOVE_TO* - возвращаем исходную команду как есть.
    return command

def send_to_arduino_and_get_response(command):
    """
    Отправка команды в Arduino с последующим чтением
    единственной строки-ответа.
    """
    print(f"[SERVER] Принята команда: {command}")
    
    # Для MOVE_TO / MOVE_TO_ROBOT трансформируем
    if command.startswith("MOVE_TO"):
        command = transform_command(command)
    
    with serial_lock:
        ser.write((command + '\n').encode())
        while True:
            response = ser.readline().decode().strip()
            if response:
                print(f"[SERVER] Ответ от Arduino: {response}")
                return response
            else:
                time.sleep(0.1)

while True:
    try:
        mode = opros()
        if mode in [0, 1]:
            break
    except:
        pass

# 0 — индекс камеры (если у тебя одна камера, обычно 0)
cap = cv2.VideoCapture(mode)

if not cap.isOpened():
    print("Не удалось открыть камеру")
    exit()

ser = find_arduino_port()
if ser is None:
    exit()

# Для гарантированной последовательности обращений к ser введём блокировку
serial_lock = threading.Lock()

while counter > 0:
    ret, frame = cap.read()  # считываем кадр
    if not ret:
        print("Не удалось получить кадр")
        break

    frame_rotate = cv2.rotate(frame, cv2.ROTATE_180)

    all_good_markers = []
    all_bad_markers = []
    marker_angles_of_bad = []
    marker_angles_of_good = []
    px_ratio = a = b = 0

    good_square_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
    bad_square_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    parameters = aruco.DetectorParameters()

    frame_gray = cv2.cvtColor(frame_rotate, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(frame_gray, 127, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    good_coordination, good_ids, _ = aruco.detectMarkers(frame_gray, good_square_dict, parameters=parameters)
    bad_coordination, bad_ids, _ = aruco.detectMarkers(frame_gray, bad_square_dict, parameters=parameters)

    for cnt in contours:
        # Аппроксимируем контур многоугольником
        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        # Если контур имеет 4 вершины — возможно прямоугольник
        if len(approx) == 4:
            # Дополнительно можно проверить, что контур выпуклый и площадь больше минимальной
            if cv2.isContourConvex(approx) and cv2.contourArea(approx) > 1000:
                # Рисуем контур
                cv2.drawContours(frame_rotate, [approx], 0, (0, 255, 0), 3)

    if good_ids is not None:
        # print(f"Обнаружены маркеры с ID: {good_ids.flatten()}")
        # Отрисовка контуров вокруг найденных маркеров
        aruco.drawDetectedMarkers(frame_rotate, good_coordination, good_ids)

        for i in range(len(good_ids)):
            # Получаем углы текущего маркера
            marker_corners = good_coordination[i].reshape((4, 2))
            
            # Вычисляем вектор между верхними углами
            dx = marker_corners[1][0] - marker_corners[0][0]
            dy = marker_corners[1][1] - marker_corners[0][1]
            
            # Вычисляем угол в градусах
            angle_deg = degrees(arctan2(dy, dx))
            
            # Добавляем ID маркера и угол в список
            marker_angles_of_good.append(angle_deg)
            
            # Выводим информацию для текущего маркера
            # print(f"Маркер ID {good_ids[i][0]}: угол поворота {angle_deg:.2f}°")


        # Объединение всех хороших точек. В том числе для дальнейшей калибровки.

        for array in good_coordination:
            all_good_markers.append(center_trapation(array))

        px_ratio, a, b = calibration(300, all_good_markers, good_ids)


    if bad_ids is not None:
        # print(f"Обнаружены маркеры с ID: {bad_ids.flatten()}")
        # Отрисовка контуров вокруг найденных маркеров

        for i in range(len(bad_ids)):
            # Получаем углы текущего маркера
            marker_corners = bad_coordination[i].reshape((4, 2))
            
            # Вычисляем вектор между верхними углами
            dx = marker_corners[1][0] - marker_corners[0][0]
            dy = marker_corners[1][1] - marker_corners[0][1]
            
            # Вычисляем угол в градусах
            angle_deg = degrees(arctan2(dy, dx))
            
            # Добавляем ID маркера и угол в список
            marker_angles_of_bad.append(angle_deg)
            
            # Выводим информацию для текущего маркера
            # print(f"Маркер ID {bad_ids[i][0]}: угол поворота {angle_deg:.2f}°")

        aruco.drawDetectedMarkers(frame_rotate, bad_coordination, bad_ids)

        for array in bad_coordination:
            all_bad_markers.append(center_trapation(array))

    # короче теперь все наши центры превращаем в словари, 
    # в которых указан тип маркера и его координаты для манипулятора

    dict_good_markers = coordinations(all_good_markers, good_ids, marker_angles_of_good, px_ratio, a, b)
    dict_bad_markers = coordinations(all_bad_markers, bad_ids, marker_angles_of_bad, px_ratio, a, b)
    
    cv2.imshow('USB камера', frame_rotate)  # показываем кадр

    # Нажми 'q' чтобы выйти
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # Освобождаем ресурсы
        cap.release()
        cv2.destroyAllWindows()
        break

    if time() - last_time > 1.0:
        print(f'У вас осталось {counter} секунд(-ы)!')
        counter -= 1
        last_time = time()

    # time.sleep(2.5)

print(dict_good_markers)
print(dict_bad_markers)

if __name__ == "__main__":
    # Пример функции для отправки команд (замените на свою реализацию)
    
    handler = MarkerHandler()
    processing_results = handler.process_markers(dict_good_markers, dict_bad_markers)
    
    # Вывод результатов обработки
    for result in processing_results:
        print(result)