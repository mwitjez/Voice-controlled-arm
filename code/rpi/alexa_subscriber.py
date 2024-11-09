import json
import time
from awscrt import io, mqtt, auth, http
from awsiot import mqtt_connection_builder
import sys
from uuid import uuid4
import serial
import ikpy.chain
import numpy as np
import freenect
import cv2
import json

endpoint = ""
port = 8883
root_ca = "AmazonRootCA1.pem"
key = ""
cert = ""
client_id = "pi-" + str(uuid4())
topic = "voice"

robot_chain = ikpy.chain.Chain.from_urdf_file("robot.URDF")
current_orientation = [0, 0, 0, np.deg2rad(90), 0]
current_pos = robot_chain.forward_kinematics(current_orientation)[:3, 3]


def get_video():
    array, _ = freenect.sync_get_video()
    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    return array


def get_depth():
    array, _ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array


# Callback when connection is accidentally lost.
def on_connection_interrupted(connection, error, **kwargs):
    print("Connection interrupted. error: {}".format(error))


# Callback when an interrupted connection is re-established.
def on_connection_resumed(connection, return_code, session_present, **kwargs):
    print("Connection resumed. return_code: {} session_present: {}".format(
        return_code, session_present))

    if return_code == mqtt.ConnectReturnCode.ACCEPTED and not session_present:
        print("Session did not persist. Resubscribing to existing topics...")
        resubscribe_future, _ = connection.resubscribe_existing_topics()

        # Cannot synchronously wait for resubscribe result because we're on the connection's event-loop thread,
        # evaluate result with a callback instead.
        resubscribe_future.add_done_callback(on_resubscribe_complete)


def calculate_angles(movement_value, direction):
    # funkcja oblicza kąty robota na podstawie zadanej pozycji
    global current_orientation
    if direction == "up":
        target_pos = [
            current_pos[0], current_pos[1], current_pos[2] + movement_value
        ]
        target_orentation = robot_chain.inverse_kinematics(target_pos)
    elif direction == "down":
        target_pos = [
            current_pos[0], current_pos[1], current_pos[2] - movement_value
        ]
        target_orentation = robot_chain.inverse_kinematics(target_pos)
    elif direction == "forward" or direction == "forwards":
        target_pos = [
            current_pos[0], current_pos[1] - movement_value, current_pos[2]
        ]
        target_orentation = robot_chain.inverse_kinematics(target_pos)
    elif direction == "backward" or direction == "backwards":
        target_pos = [
            current_pos[0], current_pos[1] + movement_value, current_pos[2]
        ]
        target_orentation = robot_chain.inverse_kinematics(target_pos)

    previous_orientation = current_orientation
    current_orientation = target_orentation
    return target_pos, previous_orientation


def open_action():
    cmd = "open"
    arduino.write(cmd.encode())
    print("opening...")


def close_action():
    cmd = "close"
    arduino.write(cmd.encode())
    print("closing...")


def stop_action():
    cmd = "stop"
    arduino.write(cmd.encode())
    print("stoping...")


def move_base_action(degrees, direction):
    cmd = "rotate" + str(degrees) + str(direction)
    arduino.write(cmd.encode())
    print("moving...")


def move_arm_action(value, direction):
    global current_orientation, current_pos
    value = float(value) / 10
    target_pos, previous_orientation = calculate_angles(value, direction)
    current_pos = robot_chain.forward_kinematics(current_orientation)[:3, 3]
    print(np.rad2deg(current_orientation))
    # czy kąty robota nie są przekraczane (robot nie wjedzie sam w siebie)
    # czy punkt jest w zasięgu robota z przybliżeniem do 4 miejsca po przecinku
    if np.rad2deg(current_orientation[2]) < 85 and np.rad2deg(
            current_orientation[2]) > -30 and np.rad2deg(
                current_orientation[3]) < 130 and np.rad2deg(
                    current_orientation[3]) > -90 and np.all(
                        np.isclose(current_pos,
                                   target_pos,
                                   rtol=1e-04,
                                   atol=1e-04,
                                   equal_nan=False)):

        angle_joint1 = np.rad2deg(previous_orientation[2] -
                                  current_orientation[2])
        angle_joint2 = np.rad2deg(previous_orientation[3] -
                                  current_orientation[3])

        print(angle_joint1)
        print(angle_joint2)
        cmd_joint1 = "move_joint1" + str(int(round(angle_joint1)))
        arduino.write(cmd_joint1.encode())
        time.sleep(0.8)
        cmd_joint2 = "move_joint2" + str(int(round(angle_joint2)))
        arduino.write(cmd_joint2.encode())
        print("moving...")
    else:
        print("Wrong angle or can't reach position")

        print(np.rad2deg(current_orientation))
        angle_joint1 = np.rad2deg(previous_orientation[2] -
                                  current_orientation[2])
        angle_joint2 = np.rad2deg(previous_orientation[3] -
                                  current_orientation[3])

        print(angle_joint1)
        print(angle_joint2)
        # powrót do poprzedniej pozycji w przypadku zbyt dużej wartości kąta
        current_orientation = previous_orientation
        current_pos = robot_chain.forward_kinematics(current_orientation)[:3,
                                                                          3]


def getContours(img, imgResult):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_NONE)
    x, y, w, h = 0, 0, 0, 0
    for countour in contours:
        area = cv2.contourArea(countour)
        if area > 70:
            cv2.drawContours(imgResult, countour, -1, (255, 0, 0), 3)
            peri = cv2.arcLength(countour, True)
            approx = cv2.approxPolyDP(countour, 0.02 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)
    return x + w // 2, y, imgResult


def get_depth_to_obj(depth, x_pos, y_pos):
    sum_depth = 0
    number_of_samples = 0
    area_size = 2  # wiekośc obszaru przeszukiwania
    for x in range(x_pos - area_size, x_pos + area_size):
        for y in range(y_pos - area_size, y_pos + area_size):
            point_depth = depth[y, x]
            if point_depth != 255:
                sum_depth += point_depth
                number_of_samples += 1
            print(point_depth)

    if number_of_samples != 0:
        avg_depth = sum_depth / number_of_samples
    else:
        avg_depth = -1
    return avg_depth


def pickup_action():
    print("picking up...")
    # pobranie obrazu i informacji o odległości z kitecta
    depth = get_depth()
    img = get_video()

    # zakresy wykoywania kolorów
    lower_range = np.array([90, 71, 66])  # B G R
    upper_range = np.array([255, 255, 255])

    # przetwarzanie obrazu- kolor, maska, głębokość
    kernel = np.ones((5, 5), np.uint8)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_range, upper_range)
    mask = cv2.morphologyEx(mask.copy(), cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask.copy(), kernel, iterations=2)
    imgResult = img.copy()
    x_pos, y_pos, imgResult = getContours(mask, imgResult)
    # przesunięcie dla czujnika głebi
    x_shifted = x_pos + 15
    y_shifted = y_pos + 33
    depth_to_object = get_depth_to_obj(depth, x_shifted, y_shifted)
    print("depth: ", depth_to_object)
    # przy założeniu wyjściowej pozycji (0, 90, 0)!!!
    if y_pos > 100 and y_pos < 300:
        angle = int(round(330 - x_pos) * 0.385)
        print("angle:", angle)
        if angle < 0:
            # jeśli kąt jest ujemny to obiekt znajduje sie po lewej stronie ramienia
            move_base_action(np.abs(angle), "left")
        else:
            move_base_action(np.abs(angle), "right")
        time.sleep(5)
        # ustaw chwytak nad obiektem
        # 117- odległość od kamery obiektu który znajduje się pod chwytakiem
        dist_to_object = int(round((104 - depth_to_object) * 2 / 13))
        print("dist: ", dist_to_object)
        if dist_to_object < 0:
            # jeśli odległość ujemna obiekt jest z tyłu
            move_arm_action(np.abs(dist_to_object), "backward")
        else:
            move_arm_action(np.abs(dist_to_object), "forward")
        time.sleep(10)
        # zejdź do ziemi 28 cm
        move_arm_action(32, "down")
        time.sleep(10)
        # zamknij chwytak
        close_action()
        time.sleep(1)
        # podjedź do góry 28 cm
        move_arm_action(32, "up")


def better_pickup_action():
    move_base_action(30, "left")
    time.sleep(2)
    move_arm_action(34, "down")
    time.sleep(8)
    close_action()
    time.sleep(2)
    move_arm_action(34, "up")
    time.sleep(8)
    move_base_action(30, "right")


def on_resubscribe_complete(resubscribe_future):
    resubscribe_results = resubscribe_future.result()
    print("Resubscribe results: {}".format(resubscribe_results))

    for topic, qos in resubscribe_results['topics']:
        if qos is None:
            sys.exit("Server rejected resubscribe to topic: {}".format(topic))


# Callback when the subscribed topic receives a message
def on_message_received(topic, payload, **kwargs):
    print("Received a new message: ")
    print(payload)
    print("from topic: ")
    print(topic)
    print("--------------\n\n")
    payload = json.loads(payload)
    command = payload['message']
    print("Processing command: ")
    print(command)

    if command == "stop":
        stop_action()
    elif command == "open":
        open_action()
    elif command == "close":
        close_action()
    elif command == "pick":
        #pickup_action()
        better_pickup_action()
    elif command == "move_base":
        degrees = payload['degrees']
        direction = payload['direction']
        print(degrees)
        print(direction)
        move_base_action(degrees, direction)
    elif command == "move_arm":
        value = payload['value']
        direction = payload['direction']
        print(value)
        print(direction)
        move_arm_action(value, direction)


if __name__ == "__main__":

    event_loop_group = io.EventLoopGroup(1)
    host_resolver = io.DefaultHostResolver(event_loop_group)
    client_bootstrap = io.ClientBootstrap(event_loop_group, host_resolver)

    proxy_options = None

    mqtt_connection = mqtt_connection_builder.mtls_from_path(
        endpoint=endpoint,
        port=port,
        cert_filepath=cert,
        pri_key_filepath=key,
        client_bootstrap=client_bootstrap,
        ca_filepath=root_ca,
        on_connection_interrupted=on_connection_interrupted,
        on_connection_resumed=on_connection_resumed,
        client_id=client_id,
        clean_session=False,
        keep_alive_secs=30,
        http_proxy_options=proxy_options)

    print("Connecting to {} with client ID '{}'...".format(
        endpoint, client_id))
    connect_future = mqtt_connection.connect()
    # Future.result() waits until a result is available
    connect_future.result()
    print("Connected!")

    print("Subscribing to topic '{}'...".format(topic))
    subscribe_future, packet_id = mqtt_connection.subscribe(
        topic=topic, qos=mqtt.QoS.AT_LEAST_ONCE, callback=on_message_received)

    subscribe_result = subscribe_future.result()
    print("Subscribed with {}".format(str(subscribe_result['qos'])))
    print("listening...")

    with serial.Serial("/dev/ttyUSB0", 9600, timeout=1) as arduino:
        time.sleep(0.1)  #wait for serial to open
        if arduino.isOpen():
            print("{} connected!".format(arduino.port))
            try:
                while True:
                    if arduino.in_waiting > 0:
                        line = arduino.readline().decode('utf-8').rstrip()
                        print("Arduino:", line)
            except KeyboardInterrupt:
                print('interrupted!')

    # Disconnect
    print("Disconnecting...")
    disconnect_future = mqtt_connection.disconnect()
    disconnect_future.result()
    print("Disconnected!")
