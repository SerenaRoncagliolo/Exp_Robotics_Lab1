
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Thanks again! Now go create something AMAZING! :D
***
***
***
*** To avoid retyping too much info. Do a search and replace for the following:
*** github_username, repo_name, twitter_handle, email, project_title, project_description
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
<!-- [![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url] -->



<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/SerenaRoncagliolo/Exp_Robotics_Lab1">
    <img src="images/logo_orizzontale_COLORE.png" width="400" height="">
  </a>

  <h3 align="center">Assignment 1 - Behavioural Architecture</h3>
<!-- togliere commenti
  <p align="center">
    This repository contains the first assignment of the experimental robotics laboratory course 2020 at the University of Genoa.
    <br />
    <a href="https://github.com/github_username/repo_name"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/github_username/repo_name">View Demo</a>
    ·
    <a href="https://github.com/github_username/repo_name/issues">Report Bug</a>
    ·
    <a href="https://github.com/github_username/repo_name/issues">Request Feature</a>
  </p> -->
</p> 



<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project
The aim of this project is to implement control system for a robot simulating a pet, that interact with a human and moves in a discrete 2D environment.
The documentation of the project can be found in the folder _Documentation_
<p align="center">
<a>
    <img src="images/home.PNG" width="350" height="">
</a>
</p>
The human can interact with the robot by using pointing gestures or spoken commands.
The robot can assume three behaviors:

* **Sleep**: the robot returns to a defined position inside the surrounding environment, it sleeps for some time and, finally, it wakes up and assumes normal behavior;
* **Play**: the robot approaches the person, it waits for a command to follow and it finally goes back to the person;
* **Normal**: the robot moves inside the environment.

<!--
## Built With

* []()
* []()
* []() -->

## Software  Architecture

### Components Architecture

<p align="center">
<a>
    <img src="images/UML.png" width="600" height="">
</a>
</p>


**Components**  
* **Voice Command Simulator**:  this component simulates the voice command "play" of the user; the command is published as a ROS message on the topic /voice_command;
* **Gesture Simulator**: this component simulates the goal position given by the user to the robot when in play mode; the position is published as a custom ROS message on the topic /pointing_gesture; in this simulation the user give the position at random intervals;
* **Behavior Command Manager**: this component simulate the Finite State Machine (FSM) and control the switching between the robot behaviors: Normal, Sleep and Play; this component publishes the new behavior as a ROS message on the topic /behavior; the other components subscribe to this topic and change function accordingly to the given behavior. The different behavior are explained later in this report.
* **Motion**: this component moves the robot using time delays to simulate it reaching the new positions, following the given behavior from the FSM. It makes use of a 2D map whose parameters are defined in a separate class; the map class is mainly used to save the updated values of the actual position everytime the behavior changes and a new function is executed. When in NORMAL state, it makes the robot moving randomly across the map; in the SLEEP state, the robot goes to the home position and rest theere until it goes back to normals; in the PLAY state, the robot goes first to the user, it waits for a goal position and then reaches it.
  
### State Machine

<p align="center">
<a>
    <img src="images/State_Machine.png" width="400" height="">
</a>
</p>
 <ol>
<li> NORMAL BEHAVIOR: when the robot assumes this behavior, it starts moving randomly within the environment. While moving, the robot should listen to the voice commands, such as the _play_ command from the user which makes it switch to PLAY behavior. Otherwise, when it is moving, the sleep timer is activated and the robot should assume SLEEP behavior;</li>
<li> SLEEP BEHAVIOR: the robot is supposed to go to a predefined position which indicates "home position" and stop there for a giveb time interval. After a certain time, it should "wake up" and assume NORMAL behavior; </li>
<li> PLAY BEHAVIOR: the robot should perform the following actions:
  <ol>
  <li> Move to the location where the user is; </li>
  <li> Wait for the pointing gesture that specifies the new target to reach; </li>
  <li> Move to new target; </li>
  <li> Go back to the user and repeat the above actions; </li>
   </ol> 
  When a certaub number of games has been reached, the robot stops playing and returns to the NORMAL state. Otherwise, the sleeping timer is triggered, it goes into SLEEP behavior.
  
  </li>
 </ol>  

## Repository Organization

* **Documentation**: it contains the html and latex documentation of the repository generated using _Doxygen_;
* **Images**: it contains the images used in the _readme_ file;
* **launch**: it contains the launch file used to run the project;
* **msg**: it contains the custom message _IntArray_ used to publish and subscribe the position of the robot;
* **src**: it contains the python scripts describing the four components and the class Map2Dclass to represent the 2D map in which the robot moves.
 
## ROS Topics and Messages
* /behaviour → topic on which the current behaviour of the robot is published as a String by _behavior_manager.py_ and subscribed by all the components
* /voice_command → topic on which the command "start playing" given by the user is published as a String by _voice_command.py_ and subscribed by _behavior_manager.py_ 
* /pointing_gesture → topic on which the goal position of the robot is published by _pointing_gesture.py_ as an IntArray (which corresponds to have int[]) and subscribed by _motion.py_
* /actual_position_robot → topic on which the current position of the robot is published by _motion.py_ as an IntArray and subscribed by _behavior_manager.py_

## Rqt_graphs 

<p align="center">
<a>
    <img src="images/rqt.PNG" width="600" height="">
</a>
</p>

<!-- GETTING STARTED
## Getting Started

To get a local copy up and running follow these simple steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* npm
  ```sh
  npm install npm@latest -g 
  ```
 -->
### Installation

1. Clone the repository
   ```sh
   git clone https://github.com/SerenaRoncagliolo/Exp_Robotics_Lab1.git
   ```
2. Enter the workspace Exp_Robotics_Lab1 and run:
   ```sh
   catkin_make
   source devel/setup.bash
   ```
3. Launch the project:
    ```sh
   roslaunch Exp_Robotics_Lab1 petbehavior.launch
   ```

<!-- USAGE EXAMPLES -->
## Usage

Here a short [video](https://github.com/github_username/repo_name/issues) showing the result of the project when running: 



<!-- ROADMAP 
## Roadmap

See the [open issues](https://github.com/github_username/repo_name/issues) for a list of proposed features (and known issues).-->



<!-- CONTRIBUTING 
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request -->



<!-- LICENSE 
## License

Distributed under the MIT License. See `LICENSE` for more information. -->



<!-- CONTACT -->
## Contact

Serena Roncagliolo - S4233330@studenti.unige.it

Project Link: [https://github.com/SerenaRoncagliolo/Exp_Robotics_Lab1](https://github.com/SerenaRoncagliolo/Exp_Robotics_Lab1)



<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

* []()
* []()
* []()





<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/github_username/repo.svg?style=for-the-badge
[contributors-url]: https://github.com/github_username/repo/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/github_username/repo.svg?style=for-the-badge
[forks-url]: https://github.com/github_username/repo/network/members
[stars-shield]: https://img.shields.io/github/stars/github_username/repo.svg?style=for-the-badge
[stars-url]: https://github.com/github_username/repo/stargazers
[issues-shield]: https://img.shields.io/github/issues/github_username/repo.svg?style=for-the-badge
[issues-url]: https://github.com/github_username/repo/issues
[license-shield]: https://img.shields.io/github/license/github_username/repo.svg?style=for-the-badge
[license-url]: https://github.com/github_username/repo/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/github_username
