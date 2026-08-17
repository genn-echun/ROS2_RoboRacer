
import eventlet
import socketio

# async_mode='eventlet' is the whole point of this probe -- it is what makes
# engine.io advertise the websocket upgrade in its OPEN packet.
sio = socketio.Server(async_mode='eventlet')

n = [0]

@sio.on('connect')
def connect(sid, environ):
    print('CONNECTED', sid, flush=True)

@sio.on('disconnect')
def disconnect(sid):
    print('DISCONNECTED', sid, 'after', n[0], 'events', flush=True)

@sio.on('Bridge')
def bridge(sid, data):
    if not data:
        print('NO DATA', flush=True)
        return
    n[0] += 1
    if n[0] % 20 == 0:
        print('events', n[0], 'keys', len(data) if isinstance(data, dict) else -1, flush=True)
    # Same payload the ROS 2 bridge sends, and the same shape the devkit's
    # F1TENTH.generate_commands() produces: {'<id> Throttle': str, '<id> Steering': str}.
    # Nonzero throttle so a working autonomous mode visibly drives the car.
    sio.emit('Bridge', data={'V1 Throttle': '0,1', 'V1 Steering': '0'})

if __name__ == '__main__':
    # socketio.Middleware with no second argument serves the Socket.IO endpoint
    # alone -- the devkit passes a Flask app only to also serve HTTP routes,
    # which this probe does not need.
    app = socketio.Middleware(sio)
    print('listening on :4567 (eventlet)', flush=True)
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
