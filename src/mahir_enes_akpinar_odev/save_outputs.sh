#!/usr/bin/env bash
set -euo pipefail

echo "--------------------------------------"

# ============================
# 1) Mod Algılama
# ============================
MODE_RAW_HIDE=$(ros2 param get /turtlesim_square control_mode --hide-type 2>/dev/null || true)

if [[ -n "${MODE_RAW_HIDE}" ]]; then
  MODE="${MODE_RAW_HIDE}"
else
  MODE_RAW=$(ros2 param get /turtlesim_square control_mode 2>/dev/null || true)
  # Örn: "String value: open" -> 3. kelimeyi al
  MODE="$(echo "${MODE_RAW}" | awk '{print $3}' | xargs)"
fi

# Varsayılan kontrol
MODE="$(echo "${MODE}" | tr '[:upper:]' '[:lower:]')"
if [[ "${MODE}" != "open" && "${MODE}" != "closed" ]]; then
  MODE="unknown"
fi

# ============================
# 2) Klasör ve Yol Ayarları
# ============================
OUTPUT_DIR="outputs/${MODE}_loop"
mkdir -p "${OUTPUT_DIR}"
cd "${OUTPUT_DIR}"

echo "Çalışma modu: ${MODE}"
echo "Çıktılar: $(pwd)"
echo "--------------------------------------"

# ============================
# 3) Node ve Topic Bilgileri
# ============================
ros2 node list | tee node_list.txt
ros2 topic list | tee topic_list.txt

# ============================
# 4) Mesaj Arayüzleri
# ============================
: > interfaces.txt
ros2 interface show geometry_msgs/msg/Twist | tee -a interfaces.txt
ros2 interface show turtlesim/msg/Pose | tee -a interfaces.txt

# ============================
# 5) cmd_vel frekansı
# ============================
echo "" > topic_hz.txt
echo "cmd_vel frekans ölçülüyor (3 saniye)..." | tee -a topic_hz.txt
timeout 3 ros2 topic hz /turtle1/cmd_vel | tee -a topic_hz.txt || true

# ============================
# 6) Pose örneği (ilk mesaj)
# ============================
sleep 1
ros2 topic echo -n 1 /turtle1/pose | tee pose_sample.txt

echo "--------------------------------------"
echo "Tüm dosyalar kaydedildi!"
echo "✅ Konum: $(pwd)"
echo "--------------------------------------"
