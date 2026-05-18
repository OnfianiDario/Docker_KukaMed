# Usa l'immagine ufficiale che contiene già Noetic, Foxy e il ros1_bridge compilato
FROM osrf/ros:foxy-ros1-bridge

# Soluzione infallibile: scarica la chiave ufficiale di ROS direttamente da GitHub
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654 || \
    (apt-get update || true && apt-get install -y curl && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -)

# Installa catkin_tools e altre dipendenze base
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Configura il workspace ROS 1
WORKDIR /ros1_ws
COPY ./src ./src

# Compila il pacchetto ROS 1
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# Copia lo script di avvio
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Automatizza il source di ROS 1 Noetic e del workspace per l'utente root
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /ros1_ws/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
