import { Routes, Route } from 'react-router-dom';
import { useEffect } from 'react';
import Home from './pages/Home.jsx';
import Map from './pages/Map.jsx';
import Settings from './pages/Settings.jsx';

function App() {
  useEffect(() => {
    const preventScroll = (e) => {
      e.preventDefault();
    };

    document.addEventListener("touchmove", preventScroll, { passive: false });

    return () => {
      document.removeEventListener("touchmove", preventScroll);
    };
  }, []);

  return (
    <>
      <Routes>
        <Route path="/" element=<Home /> />
        <Route path="/map" element=<Map /> />
        <Route path="/settings" element=<Settings /> />
      </Routes>
    </>
  )
}

export default App
