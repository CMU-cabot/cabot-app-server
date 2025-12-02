import json
import socketio
from collections import defaultdict
from flask import Flask, jsonify, render_template, request
import common
import tcp
import tour_manager


class WebUI:

    SIMPLE_LAST_EVENTS = {
        'cabot_name',
        'cabot_version',
        'elevator_settings',
        'device_status',
        'system_status',
        'battery_status',
    }

    SIMPLE_HISTORY_EVENTS = {
        'touch',
        'destination',
        'location',
        'manage_cabot',
        'navigate',
        'speak',
        'log',
    }

    IGNORE_EVENTS = {'req_version', 'req_name', 'heartbeat', 'camera_image_request', 'camera_image', 'camera_orientation'}

    def __init__(self, server: tcp.CaBotTCP):
        app: Flask = server.app
        sio: socketio.Server = server.sio
        manage_cabot_char = server.manage_cabot_char
        self.last_data = defaultdict(list)
        self.tour_manager = tour_manager.TourManager()

        @app.route('/')
        def index():
            return render_template("index.html")

        @app.route('/last_data/')
        def last_data():
            key = request.args.get('key')
            return jsonify({key: self.last_data.get(key, [])}) if key else jsonify(self.last_data)

        @app.route('/directory/')
        def directory():
            return jsonify(self.tour_manager.format_directories())

        @app.route('/publish/', methods=['POST'])
        def speak():
            body = request.get_json()
            event = body.get("event")
            if not event:
                return jsonify({'error': 'event parameter is required'}), 400

            data = body.get("data")
            kwargs = body.get("kwargs", {})
            common.logger.info(f"/publish/ event={event}, data={data}, kwargs={kwargs}")
            sio.emit(event, data, **kwargs)
            return jsonify({'status': 'ok'})

        @app.route('/manage/', methods=['POST'])
        def manage():
            body = request.get_json()
            data = body.get("data")
            if data is None:
                return jsonify({'error': 'data parameter is required'}), 400

            common.logger.info(f"/manage/ data={data}")
            manage_cabot_char.callback(0, data.encode("utf-8"))
            return jsonify({'status': 'ok'})

        # Socket.IO Wrappers
        original_emit = sio.emit
        original_handler = sio._handle_event_internal

        def emit_wrap(event, data=None, to=None, **kw):
            self._track_event(event, data, emit=True)
            return original_emit(event, data=data, to=to, **kw)

        def handler_wrap(server, sid, eio_sid, raw_data, namespace, id):
            try:
                if isinstance(raw_data, list) and len(raw_data) == 2:
                    event, payload = raw_data
                    self._track_event(event, payload)
            except Exception as e:
                common.logger.error(f"handler_wrap error: {e}")

            return original_handler(server, sid, eio_sid, raw_data, namespace, id)

        sio.emit = emit_wrap
        sio._handle_event_internal = handler_wrap

        self.tour_manager.load()
        common.logger.info("WebUI listening...")

    def _track_event(self, event, payload, emit=False):
        if isinstance(payload, list) and len(payload) == 1:
            payload = payload[0]

        if event == 'share':
            if not isinstance(payload, str):
                common.logger.info(f"Unexpected share data {payload}")
                return
            data = json.loads(payload)
            event_type = f"share.{data.get('type')}"
            value = data.get('value')

            if event_type == 'share.Speak':
                if isinstance(value, str) and value and not emit:
                    lst = self.last_data[event_type]
                    lst.append(value)
                    self.last_data[event_type] = lst[-10:]
                return

            if event_type == 'share.ChatStatus':
                if isinstance(value, str):
                    messages = json.loads(value).get('messages')
                    if messages:
                        existing = self.last_data[event_type]
                        for msg in messages:
                            for old in existing:
                                if old.get('id') == msg.get('id'):
                                    old.update(msg)
                                    break
                            else:
                                existing.append(msg)
                    else:
                        self.last_data[event_type].clear()
                return

            try:
                value = json.loads(value)
            except Exception:
                pass
            self.last_data[event_type] = [value]
            return

        if event in self.SIMPLE_LAST_EVENTS:
            try:
                payload = json.loads(payload)
            except Exception:
                pass
            self.last_data[event] = [payload]
            return

        if event in self.SIMPLE_HISTORY_EVENTS:
            try:
                payload = json.loads(payload)
            except Exception:
                pass
            lst = self.last_data[event]
            lst.append(payload)
            self.last_data[event] = lst[-10:]
            return

        if event not in self.IGNORE_EVENTS:
            common.logger.info(f"[IGNORE handle_event] event={event}, data={payload}")
