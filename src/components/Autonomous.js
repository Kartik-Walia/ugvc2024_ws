import React, { useEffect, useState } from 'react';
import './Autonomous.css';
import axios from 'axios';

const Autonomous = () => {
  const [longitude, setLongitude] = useState('');
  const [latitude, setLatitude] = useState('');
  const [history, setHistory] = useState([]);
  const [screenColor, setScreenColor] = useState('red');    // To manage metal detection
  const [switchStates, setSwitchStates] = useState({
    main: false,
    ins: false,
    gps: false,
    sensor: false,
    camera: false,
  });
  const [baseBatteryLevel, setBaseBatteryLevel] = useState(null);
  const [roverBatteryLevel, setRoverBatteryLevel] = useState(null);
  const [roverBatteryIsPlugged, setRoverBatteryIsPlugged] = useState(null);

  const handleAddCoordinate = () => {
    if (longitude && latitude && !isNaN(longitude) && !isNaN(latitude)) {
      setHistory([...history, { longitude, latitude }]);

      // Allow user to easily enter new coordinates without having to manually delete the old values
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

  // Toggles state of a given switch by name
  const toggleSwitch = (switchName) => {
    setSwitchStates((prevState) => ({
      ...prevState,
      [switchName]: !prevState[switchName],
    }));
  };

  useEffect(() => {
    const updateBaseBatteryStatus = (battery) => {
      setBaseBatteryLevel(Math.floor(battery.level * 100));
      
      battery.addEventListener('levelchange', () => {
        setBaseBatteryLevel(Math.floor(battery.level * 100));
      });
    };

    navigator.getBattery().then(updateBaseBatteryStatus);
  }, []);

  useEffect(() => {
    const fetchRoverBatteryStatus = async () => {
      try {
        const response = await axios.get('http://192.168.0.106:5000/battery');
        const data = response.data;
        if (data.percent !== undefined) {
          setRoverBatteryLevel(Math.floor(data.percent));
          setRoverBatteryIsPlugged(data.plugged);
        } else {
          setRoverBatteryLevel('Error');
          setRoverBatteryIsPlugged(false);
        }
      } catch (error) {
        console.error('Error fetching battery status:', error);
        setRoverBatteryLevel('Error');
        setRoverBatteryIsPlugged(false);
      }
    };

    fetchRoverBatteryStatus();

    const intervalId = setInterval(fetchRoverBatteryStatus, 60000); // Fetch every 60 seconds

    return () => clearInterval(intervalId);
  }, []);

  return (
    <div className="dashboard">
      <div className="header">
        <h1 className="title">AGNI CONTROL SYSTEM</h1>
        <div className='TP'>
          <div className="tp-link">TP LINK Connection</div>
          <div className="status">
            <div className="status-item-2 connected">Connected</div>
            <div className="status-item-2 not-connected">Not Connected</div>
          </div>
        </div>
      </div>

      <div className="body">
        <div className="sidebar">
          <div className="switches">
            <h3>Switches</h3>
            {['main', 'lidar', 'gps', 'ins', 'camera'].map((switchName) => (
              <>
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
              <div className='line' />
              </>
            ))}
          </div>
          <div className="screen" style={{ backgroundColor: screenColor }}></div>
          <div className="metal-detection">
            {screenColor === 'green' ? 'Metal Detected' : 'Metal Not Detected'}
          </div>
          <button className="toggle-button" onClick={handleScreenToggle}>
            Toggle Screen Color
          </button>
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
              <label htmlFor="latitude" className='label-lat'>Latitude:</label>
              <input
                type="text"
                id="latitude"
                value={latitude}
                onChange={(e) => setLatitude(e.target.value)}
              />
              <label htmlFor="longitude" className='label-long'>Longitude:</label>
              <input
                type="text"
                id="longitude"
                value={longitude}
                onChange={(e) => setLongitude(e.target.value)}
              />
            </div>
            <div className="form-group-2">
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
            <p>28°45'11.2"N</p>
            <p>77°07'04.2"E</p>
          </div>
          <div className="status-item">
            <h4>Laptop Battery (Base)</h4>
            <p>{baseBatteryLevel !== null ? `${baseBatteryLevel}%` : 'Loading...'}</p>
          </div>
          <div className="status-item">
            <h4>Laptop Battery (Rover)</h4>
            <p>{roverBatteryLevel !== null ? `${roverBatteryLevel}%` : 'Loading...'}</p>
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
