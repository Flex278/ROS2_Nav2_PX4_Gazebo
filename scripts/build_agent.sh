#!/usr/bin/env bash
# ============================================================
# build_agent.sh — сборка Micro XRCE DDS Agent (мост uORB <-> ROS2).
# Пакета micro-xrce-dds-agent в репозиториях Ubuntu Noble нет, поэтому
# собираем из исходников. Superbuild сам подтянет совместимый FastDDS 2.12.
# ============================================================
set -e

AGENT_VERSION="${1:-v2.4.2}"
AGENT_SRC="/opt/Micro-XRCE-DDS-Agent"

if [ ! -d "$AGENT_SRC/.git" ]; then
  git clone --branch "$AGENT_VERSION" \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_SRC"
fi
cd "$AGENT_SRC"

# FastDDS 2.12.x — это была ветка сопровождения; eProsima её удалила (EOL),
# поэтому супербилд падает с "invalid reference: 2.12.x". Заменяем на последний
# тег 2.12.x, совместимый с FastDDS 2.12 (API агента v2.4.2).
if [ -f CMakeLists.txt ]; then
  sed -i 's/set(_fastdds_tag 2\.12\.x)/set(_fastdds_tag v2.12.2)/' CMakeLists.txt
fi

# FastDDS 2.12.2 не компилируется с GCC 13 (Ubuntu Noble) из-за пропавшего
# <cstdint> в модуле statistics (typesv1.cxx). Модуль агента не нужен — отключаем.
if [ -f cmake/SuperBuild.cmake ]; then
  sed -i 's/-DSHM_TRANSPORT_DEFAULT:BOOL=OFF/-DSHM_TRANSPORT_DEFAULT:BOOL=OFF\n                -DFASTDDS_STATISTICS:BOOL=OFF/' cmake/SuperBuild.cmake
fi

mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DUAGENT_BUILD_EXECUTABLE=ON \
      -DUAGENT_FAST_PROFILE=ON \
      -DUAGENT_CED_PROFILE=OFF \
      -DUAGENT_P2P_PROFILE=OFF \
      -DUAGENT_DISCOVERY_PROFILE=OFF \
      ..
make -j"$(nproc)"
make install
ldconfig /usr/local/lib

echo ">>> MicroXRCEAgent: $(command -v MicroXRCEAgent || echo /usr/local/bin/MicroXRCEAgent)"
