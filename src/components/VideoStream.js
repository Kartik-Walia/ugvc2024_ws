import React, { useEffect, useRef } from 'react';

function VideoStream() {
  const canvasRef = useRef(null);
  const websocketRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const websocket = new WebSocket('ws://172.20.10.9:5000');

    websocketRef.current = websocket;

    websocket.onmessage = (event) => {
      const arrayBuffer = event.data;
      const blob = new Blob([arrayBuffer], { type: 'image/jpeg' });
      const url = URL.createObjectURL(blob);
      const img = new Image();
      img.onload = () => {
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
        URL.revokeObjectURL(url);
      };
      img.src = url;
    };

    return () => {
      websocket.close();
    };
  }, []);

  return <canvas ref={canvasRef} width={640} height={480} />;
}

export default VideoStream;
