import React from 'react';
import Manual from './components/Manual';
import { BrowserRouter, Routes, Route, Link } from 'react-router-dom';
import './App.css';

const App = () => {
  return (
    <BrowserRouter>
      <Routes>
        <Route path='/manual' element={<Manual />} />
      </Routes>
    </BrowserRouter>
  )
};

export default App;