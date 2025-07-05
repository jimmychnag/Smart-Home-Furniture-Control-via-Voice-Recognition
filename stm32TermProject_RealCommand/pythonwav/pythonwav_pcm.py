import serial
import wave
import struct
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
from pydub import AudioSegment

# ==== 錄音設定 ====
ser = serial.Serial('COM8', 115200*4, timeout=1)  # 確保跟 STM32 UART 一致

duration = 1               # 錄製 1 秒
sample_rate = 16000        # 採樣率
num_samples = duration * sample_rate
pcm_data = []

print(f"開始錄製 {duration} 秒音頻，採樣率 {sample_rate} Hz...")

# ==== 錄製資料 ====
while len(pcm_data) < num_samples:
    raw = ser.read(3)  # 每次讀 3 bytes（24-bit）
    if len(raw) == 3:
        # 小端轉成 24-bit int（不壓縮）
        val = raw[0] | (raw[1] << 8) | (raw[2] << 16)

        # Sign-extend to 32-bit
        if val & 0x800000:
            val |= ~0xFFFFFF

        pcm_data.append(val)

    if len(pcm_data) % 10000 == 0:
        print(f"已錄製 {len(pcm_data)} / {num_samples} 樣本...")

print("錄製完成，寫入 WAV 檔案...")

# ==== 儲存為 24-bit WAV ====
with wave.open("output_24bit.wav", 'wb') as wav_file:
    wav_file.setnchannels(1)
    wav_file.setsampwidth(3)  # 3 bytes = 24-bit
    wav_file.setframerate(sample_rate)

    # 將 int32 轉成 24-bit 小端格式
    frames = bytearray()
    for val in pcm_data:
        val &= 0xFFFFFF  # 保留低 24-bit
        frames += struct.pack('<i', val)[0:3]  # 取前 3 bytes（小端）

    wav_file.writeframes(frames)

print("完成！已儲存為 output_24bit.wav")

# ==== 波形與 FFT 分析 ====
print("開始做 FFT 分析...")

pcm_np = np.array(pcm_data, dtype=np.int32)

# 時間軸
times = np.arange(len(pcm_np)) / sample_rate

# 畫時間波形
plt.figure(figsize=(12, 4))
plt.plot(times, pcm_np)
plt.title("Time Domain Signal (24-bit)")
plt.xlabel("Time [s]")
plt.ylabel("Amplitude")
plt.grid()
plt.tight_layout()
plt.show()

# 做 FFT
N = len(pcm_np)
yf = fft(pcm_np)
xf = fftfreq(N, 1 / sample_rate)

# 只取正頻率
idx = np.where(xf >= 0)

plt.figure(figsize=(12, 4))
plt.plot(xf[idx], np.abs(yf[idx]))
plt.title("Frequency Domain (FFT, 24-bit)")
plt.xlabel("Frequency [Hz]")
plt.ylabel("Magnitude")
plt.grid()
plt.tight_layout()
plt.show()


# 讀取 24-bit 音訊
audio = AudioSegment.from_file("output_24bit.wav")

# 設定為 16-bit (2 bytes per sample)
audio_16bit = audio.set_sample_width(2)

# 匯出成 16-bit PCM WAV 檔
audio_16bit.export("output_16bit_three2.wav", format="wav")

print("✅ 完成！已儲存為 output_16bit.wav")