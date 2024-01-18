import subprocess
import datetime

# 作者信息
__author__ = "Teddy"
# 邮箱
__email__ = "teddy@pixmoving.net"
# 脚本编写时间
__date__ = datetime.date.today().strftime("%Y-%m-%d")
# 用途描述
__purpose__ = "调整 PulseAudio 音量的简单命令行工具"
# 所属公司
__company__ = "Guizhou Hankais Intelligent Technology Co."


def set_volume(volume_percentage):
    # 将百分比转换为范围在 0 到 65536 之间的整数值
    volume_value = int(volume_percentage * 65536 / 100)
    command = f"pacmd set-sink-volume 0 {volume_value}"
    try:
        subprocess.run(command, shell=True, check=True)
        print(f"音量已调整为 {volume_percentage}%")
    except subprocess.CalledProcessError as e:
        print(f"错误：{e}")

def main():
    while True:
        try:
            # 获取用户输入
            user_input = input("请输入音量级别(0-100,输入 q 退出）： ")

            # 如果用户输入 'q'，退出循环
            if user_input.lower() == 'q':
                break

            # 尝试将用户输入转换为整数
            volume = int(user_input)

            # 确保音量在有效范围内
            if 0 <= volume <= 100:
                # 设置音量
                set_volume(volume)
            else:
                print("无效的音量值，请输入 0 到 100 之间的值。")
        except ValueError:
            print("无效的输入，请输入有效的数字值。")

if __name__ == "__main__":
    main()
