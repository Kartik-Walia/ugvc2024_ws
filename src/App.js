import React from 'react';
import Manual from './components/Manual';
import Autonomous from './components/Autonomous';
import { BrowserRouter, Routes, Route } from 'react-router-dom';
import VideoStream from './components/VideoStream';
import Manual2 from './components/Manual2';
import CameraStream from './components/CameraStream';

const App = () => {
  return (
    <BrowserRouter>
      <Routes>
        <Route path='/' element={<Autonomous />} />
        <Route path='/manual' element={<Manual />} />
        <Route path='/manual2' element={<Manual2 />} />
      </Routes>
    </BrowserRouter>
  )
};

export default App;