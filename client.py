import os
import sys
import time

from pynput import keyboard

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from edubotsdk.mavcar import ConnectionMethod, MavCarClient


def main():
    host = (
        input("Enter IP address of the device (empty by default): ").strip()
        or "arena-rpi-geobot.local"
    )
    port_str = input("Port (empty by default): ").strip()
    port = int(port_str) if port_str.isdigit() else 5656
    client = MavCarClient(
        ip=host, mavlink_port=port, connectionMethod=ConnectionMethod.udpout
    )
    _speed_step = 50
    control_data = {"channels": {1: 1500, 3: 1500, 4: 1500}, "color": [0, 0, 0]}

    def on_press(key):
        try:
            match key.char:
                case "w":
                    control_data["channels"][3] = 1000
                case "a":
                    control_data["channels"][4] = 1000
                case "s":
                    control_data["channels"][3] = 2000
                case "d":
                    control_data["channels"][4] = 2000
                case "e":
                    if control_data["channels"][1] <= 2000 - _speed_step:
                        control_data["channels"][1] += _speed_step
                case "q":
                    if control_data["channels"][1] >= 1000 + _speed_step:
                        control_data["channels"][1] -= _speed_step

                case "1":
                    control_data["color"] = [255, 0, 0]
                case "2":
                    control_data["color"] = [0, 255, 0]
                case "3":
                    control_data["color"] = [0, 0, 255]
                case "4":
                    control_data["color"] = [255, 255, 255]
                case "5":
                    control_data["color"] = [0, 0, 0]

            client.led_custom(
                color1=control_data["color"], color2=control_data["color"]
            )
            print(f"Color set: {control_data['color']}")

        except AttributeError:
            pass

    def on_release(key):
        try:
            match key.char:
                case "w":
                    control_data["channels"][3] = 1500
                case "a":
                    control_data["channels"][4] = 1500
                case "s":
                    control_data["channels"][3] = 1500
                case "d":
                    control_data["channels"][4] = 1500
        except AttributeError:
            pass

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    print("Client started")
    try:
        while True:
            pos = client.get_local_position_lps(get_last_received=True)
            attitude = client.get_attitude(get_last_received=True)
            print(f"LPS: {pos}\tAttitude: {attitude}")

            client.send_rc_channels(
                channel_1=control_data["channels"][1],  # скорость
                channel_3=control_data["channels"][3],  # вперед - назад
                channel_4=control_data["channels"][4],  # влево - вправо
            )

            print(f"Channels set: {control_data['channels']}")

            time.sleep(0.1)  # 20 Гц

    except KeyboardInterrupt:
        print("Interrupted")
        ans = input("Poweroff the device? Y or N: ").strip().upper()
        if ans == "Y":
            client.raspberry_poweroff_send()
            print("Shutdown")
        sys.exit()


if __name__ == "__main__":
    main()
