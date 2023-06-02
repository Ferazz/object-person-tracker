FROM ros:foxy

# To run this a secret called user_password needs to be set

RUN apt update && apt install --fix-missing -y ros-foxy-rviz2 '?name(ros-foxy-rqt.*)' 

RUN apt install -y python3-pip
RUN pip install numpy==1.20.3
RUN pip install ultralytics pyserial

RUN apt update && apt install python3

# Final setup
RUN rosdep update
RUN echo $USER_PASSWORD >> rosdep install --from-paths . --ignore-src -y

RUN apt update && apt install wget

# Oh my zsh install
RUN ln -s /workspace/paintball-person-tracker/.docker_zsh_history  /root/.zsh_history
# Install zsh ( since it looks better than /bin/bash :) )
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.5/zsh-in-docker.sh)"

# Setting upp zsh "spaceship" configuration
ENV ZSH /root/.oh-my-zsh
RUN git clone https://gist.github.com/408b6f921bcf9aa2bf23174a38d2168b.git /zsh_config 
RUN cd /zsh_config && sh ./install && mv .zshrc /root
#ENV ZSH_AUTOSUGGEST_HIGHLIGHT_STYLE "fg=#100"
CMD /bin/bash source etc/zsh/zshrc
RUN echo "\nsource \"/opt/ros/foxy/setup.zsh\"" >> /etc/zsh/zshrc
RUN ldconfig


# Create a non-root user with the same user and group IDs
RUN addgroup --gid 1000 sebastian_docker && \
    adduser --uid 1000 --ingroup sebastian_docker --disabled-password --gecos "" sebastian_docker
USER sebastian_docker


WORKDIR /workspace/paintball-person-tracker