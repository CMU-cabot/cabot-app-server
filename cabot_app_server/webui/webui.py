import json

from collections import defaultdict
from flask import Flask, jsonify, render_template, request
import socketio
import common
import tcp


class WebUI:

    def __init__(self, server: tcp.CaBotTCP):
        app: Flask = server.app
        sio: socketio.Server = server.sio
        manage_cabot_char: common.CabotManageChar = server.manage_cabot_char
        _original_emit = sio.emit
        _original_handler = sio._handle_event_internal
        self.last_data = defaultdict(list)

        @app.route('/')
        def index():
            return render_template("index.html")

        @app.route('/last_data/')
        def last_data():
            key = request.args.get('key')
            if key:
                return jsonify({key: self.last_data.get(key, [])})
            return jsonify(self.last_data)

        @app.route('/share/', methods=['POST'])
        def share():
            body = request.get_json()
            data = body.get("data")
            if data is None:
                return jsonify({'error': 'data parameter is required'}), 400
            else:
                kwargs = body.get("kwargs", {})
                common.logger.info(f"/share/ data={data}, kwargs={kwargs}")
                sio.emit('share', data, **kwargs)
                return jsonify({'status': 'ok'})

        @app.route('/manage/', methods=['POST'])
        def manage():
            body = request.get_json()
            data = body.get("data")
            if data is None:
                return jsonify({'error': 'data parameter is required'}), 400
            else:
                common.logger.info(f"/manage/ data={data}")
                manage_cabot_char.callback(0, data.encode("utf-8"))
                return jsonify({'status': 'ok'})

        def emit_wrap(event, data=None, to=None, **kw):
            track_event(event, data)
            return _original_emit(event, data=data, to=to, **kw)

        def handler_wrap(server, sid, eio_sid, raw_data, namespace, id):
            try:
                if isinstance(raw_data, list) and len(raw_data) == 2:
                    track_event(raw_data[0], raw_data[1])
                else:
                    common.logger.info(f"[IGNORE handler_wrap] data={raw_data}")
            except Exception as e:
                common.logger.error(f"handler_wrap error: {e}")
            return _original_handler(server, sid, eio_sid, raw_data, namespace, id)

        def track_event(event, payload):
            if isinstance(payload, list) and len(payload) == 1:
                payload = payload[0]
            if event == 'share' and isinstance(payload, str):
                payload = json.loads(payload)
                type = f"share.{payload.get('type')}"
                value = payload.get('value')
                if type == 'share.Speak':
                    if isinstance(value, str) and len(value) > 0:
                        self.last_data[type].append(value)
                        self.last_data[type] = self.last_data[type][-10:]
                elif type == 'share.ChatStatus':
                    if isinstance(value, str):
                        messages = json.loads(value).get('messages')
                        if messages:
                            for message in messages:
                                for m in self.last_data[type]:
                                    if m.get('id') == message.get('id'):
                                        m.update(message)
                                        break
                                else:
                                    self.last_data[type].append(message)
                        else:
                            self.last_data[type].clear()
                else:
                    self.last_data[type] = [value]
            elif event in ['destination', 'elevator_settings', 'touch', 'device_status', 'system_status', 'battery_status', 'cabot_name', 'location']:
                self.last_data[event] = [payload]
            elif event in ['req_version', 'req_name', 'heartbeat', 'manage_cabot', 'log', 'camera_image_request', 'navigate', 'speak', 'cabot_version', 'camera_image', 'camera_orientation']:
                pass
            else:
                common.logger.info(f"[IGNORE handle_event] event={event}, data={payload}")

        sio.emit = emit_wrap
        sio._handle_event_internal = handler_wrap
        common.logger.info("WebUI listening...")
