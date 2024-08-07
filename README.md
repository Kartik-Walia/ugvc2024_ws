# UGV Controller - React GUI

This repository contains the React GUI for controlling the Unmanned Ground Vehicle (UGV) both manually and autonomously. This branch focuses on providing a user interface for manual control of the rover with various options for controlling the motored wheels.

## Features

1. **Manual Control**:
   - **All 4 Wheels Together**: Control all four motored BLDC wheels simultaneously.
   - **Individual Wheel Control**: Control each of the four motored BLDC wheels separately for more precise maneuvering.

2. **Autonomous Control**: 
   - Provides an interface for autonomous operations of the rover.

## Rover Mechanism

The rover utilizes a rocker-bogie mechanism which includes:
- **2 Unmotored Wheels**: Fixed wheels that assist in terrain traversal.
- **4 Motored BLDC Wheels**: Wheels controlled via the React GUI for precise movement and navigation.

## Getting Started

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/thekartikwalia/ugvc2024_ws.git
   cd ugvc2024_ws
   git fetch -all
    ```
2. **Checkout the react_gui Branch**:
    ```bash
    git checkout react_gui
    ```
3. **Install Dependencies**:
    ```bash
    npm install
    npm install react-icons --save
    ```
4. **Run the Application**:
    Start the React development server to run the GUI locally.
    ```bash
    npm start
    ```

## Usage

- **Manual Control**:
  - **All 4 Wheels Together**:  
    Open your web browser and navigate to the [manual](http://localhost:3001/manual) route to access the React GUI for controlling all 4 wheels together simultaneously.
  - **Individual Wheel Control**: 
    Open your web browser and navigate to the [manual2](http://localhost:3001/manual2) route to access the React GUI for controlling all 4 wheels together simultaneously.

- **Autonomous Control**: 
  Open your web browser and navigate to the [home](http://localhost:3001/) route.

## Contributing

Contributions are welcome! Please create a pull request or raise an issue for any enhancements or bug fixes.

<!-- ## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details. -->

## Contact

For any queries or further information, please contact:

- Phone: [+91 9717090525](tel:+919717090525)
- Email: [business.kartikwalia@gmail.com](mailto:business.kartikwalia@gmail.com)