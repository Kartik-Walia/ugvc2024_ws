import React from 'react';
import Manual from './components/Manual';
import Autonomous from './components/Autonomous';
import { BrowserRouter, Routes, Route, Link } from 'react-router-dom';

const App = () => {
  return (
    <BrowserRouter>
      <Routes>
        <Route path='/' element={<Autonomous />} />
        <Route path='/manual' element={<Manual />} />
      </Routes>
    </BrowserRouter>
  )
};

export default App;