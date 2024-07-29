import React from 'react';

function CameraStream() {
  return (
    <div>
      <h1>Camera Stream</h1>
      <img src="http://172.20.10.6:5000/" id="stream" alt="Camera Stream" />
    </div>
  );
}

export default CameraStream;