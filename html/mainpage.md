
@mainpage Cyberpunk Mech Turtlebot: Advanced Programming for Robots

<!-- Cyberpunk Mech Turtlebot with 50% scaled image -->
<div style="text-align: center;">
  <img src="mech_turtle.jpeg" alt="Cyberpunk Mech Turtlebot" width="30%"/>
</div>

---

## 🐢 Welcome to Cyberpunk Mech Turtlebot

The **Cyberpunk Mech Turtlebot** is an advanced robotics platform designed to merge sleek cyberpunk 
aesthetics with cutting-edge technology. This project pushes the boundaries of robotics, offering 
robust hardware and modular software for urban and industrial applications.

---

## 🚀 Project Highlights
- **Rugged Cyberpunk Design**: Built to withstand dynamic environments and urban hazards.
- **Customizable Hardware**: Add-ons for surveillance, delivery, and exploration.
- **Advanced Sensor Suite**: Includes LIDAR, ultrasonic sensors, and cameras for 360-degree awareness.
- **Seamless Integration**: using Telepathy.

---

## 🛠️ Getting Started

Clone the repository and launch the project:

```bash
git clone https://github.com/arthurpellegrini/AdvancedProgramingRobot
cd cyberpunk-mech-turtlebot
```
# How to compile and commit provided c-scripts 

```bash
cd sockets
./compileme.sh
```

```bash
git add .

git commit -m "feat: TITLE" \
-m "DESCRIPTION" \
-m "Co-authored-by: nils93 <n.fandrey@gmail.com>" \
-m "Co-authored-by: Arthur PELLEGRINI <arthur.pellegrini@outlook.fr>" \
-m "Co-authored-by:  cfriedl1 <ch.friedl1010@gmail.com>" \  
-m "Co-authored-by:  Felix130899 <felixhaier1308@gmail.com>"
#only 1 author + 1 co-author 

git push
```

# Install required Json package
```bash
sudo apt-get install libjsoncpp-dev
```

# Generated Executables

* ListenOnTCPPort:
A TCPSocket that connects to a tcp-ip port. Received data will be displayed in the terminal.

* CommandHandler:
A TCPSocket that connects to a tcp-ip port. Data can be send via user input via this port.


# Setting up the connection with the robot
Cyberpunk mech turtlebot
IP: 192.168.100.5..

Password: on the back side of the router

Put the Battery and do not forgot to switch the button to turn on the robot.

Try the connection between the robot and your laptop: 
```sh
ping 192.168.100.5..
```
* Port 9998->
/odom

* Port 9997->
/scan

* Port 9999->
/cmd_vel

## ⚙️ Key Technologies
- **Hardware Components**:
  - Raspberry pi 3.
  - High-precision motors and modular sensor arrays.
- **Languages**: C++.
- **Documentation**: Doxygen for code documentation.

---

## 📜 Generated Tools and Executables
- **CommanderHandler**: Sends movement commands via TCP.
- **ListenOnTCPPort**: Displays real-time TCP data in the terminal.


---

## 🤝 How to Contribute

We welcome contributors to help shape the future of the Cyberpunk Mech Turtlebot project!

1. Fork the repository.
2. Create a feature branch.
3. Submit a pull request with your changes.

For inquiries, please don't contact us at **contact@mechturtle.turtlebot3**.

---

## 🌐 Useful Links
- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [Conventional Commits Guide](https://www.conventionalcommits.org/en/v1.0.0/)

---

## 📝 Acknowledgments
- Thanks to the open-source robotics community.
- Special recognition to our developers and sponsors for making this vision possible.

---

> *"The best way to predict the future is to invent it."*  
> – Alan Kay

