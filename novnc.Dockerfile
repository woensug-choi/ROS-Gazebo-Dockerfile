FROM ubuntu:jammy
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update -y && apt install --no-install-recommends -y xfce4 xfce4-goodies tigervnc-standalone-server novnc websockify sudo xterm init systemd snapd vim net-tools curl wget git tzdata
RUN apt update -y && apt install -y dbus-x11 x11-utils x11-xserver-utils x11-apps mesa-utils
RUN apt install software-properties-common -y
RUN add-apt-repository ppa:mozillateam/ppa -y
RUN echo 'Package: *' >> /etc/apt/preferences.d/mozilla-firefox
RUN echo 'Pin: release o=LP-PPA-mozillateam' >> /etc/apt/preferences.d/mozilla-firefox
RUN echo 'Pin-Priority: 1001' >> /etc/apt/preferences.d/mozilla-firefox
RUN echo 'Unattended-Upgrade::Allowed-Origins:: "LP-PPA-mozillateam:jammy";' | tee /etc/apt/apt.conf.d/51unattended-upgrades-firefox
RUN apt update -y && apt install -y firefox

RUN apt update -y && apt install -y xubuntu-icon-theme fonts-nanum

ENV LC_ALL=C.UTF-8
# RUN apt update && apt-get install -y locales
# RUN locale-gen ko_KR.UTF-8
# ENV LC_ALL ko_KR.UTF-8

RUN apt update -y && apt install -y uim uim-byeoru
ENV XIM=uim
ENV GTK_IM_MODULE=uim
ENV QT_IM_MODULE=uim
ENV XMODIFIERS=@im=uim
ENV UIM_CANDWIN_PROG=uim-candwin-gtk

RUN touch /root/.Xauthority
EXPOSE 5901
EXPOSE 6080
CMD bash -c "vncserver -localhost no -SecurityTypes None -geometry 1920x1080 --I-KNOW-THIS-IS-INSECURE && openssl req -new -subj "/C=KR" -x509 -days 365 -nodes -out self.pem -keyout self.pem && websockify -D --web=/usr/share/novnc/ --cert=self.pem 6080 localhost:5901 && tail -f /dev/null"

# docker build -f novnc.Dockerfile -t novnc .
# docker run -it -p 6080:6080 novnc
# http://localhost:6080/vnc.html