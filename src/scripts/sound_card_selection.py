#!/usr/bin/env python3

import simpleaudio
import pyaudio
import time

class SoundPlayer:
    def __init__(self):
        # 创建 PyAudio 对象
        self.p = pyaudio.PyAudio()
        # 获取声卡设备数量
        self.num_devices = self.p.get_device_count()
        # 定义声卡设备的标识
        self.Interior_speaker = 0  # 声卡1
        self.Exterior_speaker = 1  # 声卡2
        # 选择的声卡设备索引
        self.selected_device_index = None

    def list_usb_sound_cards(self):
        print("USB 外设的声卡列表：")
        for i in range(self.num_devices):
            device_info = self.p.get_device_info_by_index(i)
            device_name = device_info["name"]
            device_host_api = self.p.get_host_api_info_by_index(device_info["hostApi"])["name"]

            # 检查设备是否为 USB 外设
            if "USB" in device_name or "usb" in device_name:
                print(f"设备 {i}: {device_name} (Host API: {device_host_api})")

    def select_sound_card(self, sound_card):
        # 根据选择的声卡设备标识选择声卡
        for i in range(self.num_devices):
            if i == sound_card:
                self.selected_device_index = i
                break

    def play_audio(self, file_path):
        if self.selected_device_index is None:
            print("未选择声卡设备")
            return

        # 使用 simpleaudio 播放音频文件
        wave_obj = simpleaudio.WaveObject.from_wave_file(file_path)
        play_obj = wave_obj.play()

        # 等待音频播放完成
        play_obj.wait_done()

    def close(self):
        # 关闭 PyAudio 对象
        self.p.terminate()

if __name__ == "__main__":
    sound_player = SoundPlayer()
    sound_player.list_usb_sound_cards()

    # 选择声卡设备并播放音频
    sound_card = sound_player.Interior_speaker  # 选择 Interior_speaker 或 Exterior_speaker
    sound_player.select_sound_card(sound_card)

    if sound_card == sound_player.Interior_speaker:
        audio_file_path = "/home/pixbus/pix/robobus/remoteware-robobus/src/vehicle_voice_alert_system/vehicle_voice_alert_system/resource/station_list/Baiyun_park.wav"  # 替换为 Interior_speaker 的音频文件路径
    elif sound_card == sound_player.Exterior_speaker:
        audio_file_path = "/home/pixbus/pix/robobus/remoteware-robobus/src/vehicle_voice_alert_system/vehicle_voice_alert_system/resource/station_list/Guanshan_Lake_park.wav"  # 替换为 Exterior_speaker 的音频文件路径
    else:
        audio_file_path = None

    if audio_file_path:
        try:
            sound_player.play_audio(audio_file_path)
            time.sleep(10)  # 播放音频文件10秒
        except simpleaudio.exceptions.WaveObjectException as e:
            print(f"发生 simpleaudio 错误：{e}")
        finally:
            sound_player.close()
