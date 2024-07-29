import React, { useState } from 'react';
import axios from 'axios';
import { FaArrowRight, FaArrowLeft, FaArrowUp, FaArrowDown } from "react-icons/fa";
import './Manual.css';

const Manual2 = () => {
  const [speed, setSpeed] = useState(30);

  const handleClick = (url) => {
    axios.get(url)
      .then(response => {
        console.log(response);
      })
      .catch(error => {
        console.error(error);
      });
  };

  return (
    <div className="container">
      <div className="header">
        <h1 className="title">AGNI MANUAL CONTROLLER</h1>
      </div>

      <div className='speed-buttons'>
        <h1 className='head-speed'>Left Front</h1>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftfront/50')}>50</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftfront/60')}>60</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftfront/70')}>70</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftfront/80')}>80</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftfront/90')}>90</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftfront/100')}>100</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftfront/100')}>150</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftfront/100')}>200</button>
      </div>  

      <div className='speed-buttons'>
        <h1 className='head-speed'>Right Front</h1>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightfront/50')}>50</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightfront/60')}>60</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightfront/70')}>70</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightfront/80')}>80</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightfront/90')}>90</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightfront/100')}>100</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightfront/100')}>150</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightfront/100')}>200</button>
      </div>  

      <div className='speed-buttons'>
        <h1 className='head-speed'>Left Back</h1>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftback/50')}>50</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftback/60')}>60</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftback/70')}>70</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftback/80')}>80</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftback/90')}>90</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftback/100')}>100</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftback/100')}>150</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/leftback/100')}>200</button>
      </div>  

      <div className='speed-buttons'>
        <h1 className='head-speed'>Right Back</h1>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightback/50')}>50</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightback/60')}>60</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightback/70')}>70</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightback/80')}>80</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightback/90')}>90</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightback/100')}>100</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightback/100')}>150</button>
        <button className='speed-btn' onClick={() => handleClick('http://192.168.43.89:5000/speed/rightback/100')}>200</button>
      </div>  

    </div>
  );
};

export default Manual2;