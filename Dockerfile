FROM ros:jazzy

# Установка зависимостей (если нужно)
RUN apt update && apt install -y \
    python3-rpi.gpio \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Копируем workspace
COPY ./src /ros2_ws/src
WORKDIR /ros2_ws

# Собираем workspace
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install -i --from-path src --rosdistro jazzy -y && \
    colcon build

# Устанавливаем entrypoint для активации окружения
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]