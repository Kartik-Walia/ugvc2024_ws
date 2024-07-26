// src/Dashboard.js
import React, { useState } from 'react';
import './Dashboard.css';

const Dashboard = () => {
  const [longitude, setLongitude] = useState('');
  const [latitude, setLatitude] = useState('');
  const [history, setHistory] = useState([]);

  const handleAddCoordinate = () => {
    if (longitude && latitude) {
      setHistory([...history, { longitude, latitude }]);
      setLongitude('');
      setLatitude('');
    }
  };

  const handleClearHistory = () => {
    setHistory([]);
  };

  return (
    <div className="dashboard">
      <div className="header">
        <div className="com">
          <label htmlFor="device">Device</label>
          <select id="device">
            <option value="jetson-nano">Jetson Nano</option>
          </select>
        </div>
        <div className="status">
          <span className="connected">Connected</span>
          <span className="not-connected">Not Connected</span>
        </div>
      </div>
      <div className="body">
        <div className="sidebar">
          <div className="switches">
            <h3>Switches</h3>
            <div className="switch">
              <label htmlFor="switch">Switches</label>
              <button id="switch">ON</button>
            </div>
            <div className="switch">
              <label htmlFor="ins">INS</label>
              <button id="ins">OFF</button>
            </div>
            <div className="switch">
              <label htmlFor="gps">GPS</label>
              <button id="gps">ON</button>
            </div>
            <div className="switch">
              <label htmlFor="sensor">Sensor</label>
              <button id="sensor">ON</button>
            </div>
            <div className="switch">
              <label htmlFor="camera">Camera</label>
              <button id="camera">OFF</button>
            </div>
          </div>
        </div>
        <div className="main">
          <div className="coordinate-form">
            <h3>Enter Coordinates</h3>
            <div className="form-group">
              <label htmlFor="latitude">Latitude</label>
              <input
                type="text"
                id="latitude"
                value={latitude}
                onChange={(e) => setLatitude(e.target.value)}
              />
            </div>
            <div className="form-group">
              <label htmlFor="longitude">Longitude</label>
              <input
                type="text"
                id="longitude"
                value={longitude}
                onChange={(e) => setLongitude(e.target.value)}
              />
            </div>
            <button onClick={handleAddCoordinate}>Add Coordinate</button>
            <button onClick={handleClearHistory} className="clear-history">
              Clear History
            </button>
          </div>
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

export default Dashboard;
