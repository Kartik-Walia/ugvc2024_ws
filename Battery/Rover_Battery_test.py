from flask import Flask, jsonify 
from flask_cors import CORS 
import psutil 

app = Flask(__name__)
CORS(app)    # Enable CORS for all routes 

@app.route('/battery', methods=['GET'])
def get_battery_status():
    battery = psutil.sensors_battery
    if battery:
        return jsonify({
            'percent': battery.percent,
            'plugged': battery.power_plugged
        })
        
    return jsonify({'Error': 'Battery information not available'}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0')