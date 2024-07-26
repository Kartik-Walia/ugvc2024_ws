import React, { useState } from 'react';
import './Autonomous.css';

const Autonomous = () => {
  const [longitude, setLongitude] = useState('');
  const [latitude, setLatitude] = useState('');
  const [history, setHistory] = useState([]);
  const [screenColor, setScreenColor] = useState('red');
  const [switchStates, setSwitchStates] = useState({
    main: false,
    ins: false,
    gps: false,
    sensor: false,
    camera: false,
  });

  const handleAddCoordinate = () => {
    if (longitude && latitude && !isNaN(longitude) && !isNaN(latitude)) {
      setHistory([...history, { longitude, latitude }]);
      setLongitude('');
      setLatitude('');
    } else {
      alert('Please enter valid numeric coordinates.');
    }
  };

  const handleClearHistory = () => {
    setHistory([]);
  };

  const handleScreenToggle = () => {
    setScreenColor(screenColor === 'red' ? 'green' : 'red');
  };

  const toggleSwitch = (switchName) => {
    setSwitchStates((prevState) => ({
      ...prevState,
      [switchName]: !prevState[switchName],
    }));
  };

  return (
    <div className="dashboard">
      <div className="header">
        <h1 className="title">AGNI CONTROL CENTRE</h1>
        <div className="status">
          <div className="status-item connected">Connected</div>
          <div className="status-item not-connected">Not Connected</div>
        </div>
        <div className="tp-link">TP LINK connection</div>
      </div>
      <div className="body">
        <div className="sidebar">
          <div className="switches">
            <h3>Switches</h3>
            {['main', 'ins', 'gps', 'sensor', 'camera'].map((switchName) => (
              <div className="switch" key={switchName}>
                <label htmlFor={switchName}>{switchName.toUpperCase()}</label>
                <button
                  id={switchName}
                  className={switchStates[switchName] ? 'switch-on' : 'switch-off'}
                  onClick={() => toggleSwitch(switchName)}
                >
                  {switchStates[switchName] ? 'ON' : 'OFF'}
                </button>
              </div>
            ))}
          </div>
          <div className="screen" style={{ backgroundColor: screenColor }}></div>
          <button className="toggle-button" onClick={handleScreenToggle}>
            Toggle Screen Color
          </button>
          <div className="metal-detection">
            {screenColor === 'green' ? 'Metal Object Detected' : 'Metal Object Not Detected'}
          </div>
        </div>
        <div className="main">
          <div className="history">
            <h3>History</h3>
            <ul>
              {history.map((coord, index) => (
                <li key={index}>
                  Latitude: {coord.latitude}, Longitude: {coord.longitude}
                </li>
              ))}
            </ul>
          </div>
          <div className="coordinate-form">
            <h3>Enter Coordinates</h3>
            <div className="form-group">
              <label htmlFor="latitude">Latitude:</label>
              <input
                type="text"
                id="latitude"
                value={latitude}
                onChange={(e) => setLatitude(e.target.value)}
              />
              <label htmlFor="longitude">Longitude:</label>
              <input
                type="text"
                id="longitude"
                value={longitude}
                onChange={(e) => setLongitude(e.target.value)}
              />
              <button onClick={handleAddCoordinate}>Add Coordinate</button>
              <button onClick={handleClearHistory} className="clear-history">
                Clear History
              </button>
            </div>
          </div>
        </div>
        <div className="status-info">
          <div className="status-item">
            <h4>Speed</h4>
            <p>0 Km/Hr</p>
          </div>
          <div className="status-item">
            <h4>Location</h4>
            <p>28°45'11.2"N 77°07'04.2"E</p>
          </div>
          <div className="status-item">
            <h4>Battery</h4>
            <p>86%</p>
          </div>
          <div className="status-item">
            <h4>Direction</h4>
            <p>North East</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Autonomous;
