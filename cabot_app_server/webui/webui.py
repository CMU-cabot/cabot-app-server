#!/usr/bin/env python

# Copyright (c) 2025  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import base64
import json
import re
import socketio
from collections import defaultdict
from flask import Flask, Response, jsonify, render_template, request
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
        'location',
    }

    SIMPLE_HISTORY_EVENTS = {
        'touch',
        'destination',
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
        self.last_image = {}
        self.tour_manager = tour_manager.TourManager()

        @app.route('/')
        def index():
            return render_template("index.html")

        @app.route('/last_data/')
        def last_data():
            self.last_data['localize_status'] = [common.last_localize_status]
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

        @app.route('/camera_image/')
        def camera_image():
            return jsonify(
                [
                    {'image': self._get_camera_image(common.last_camera_left_image), 'position': 'left', 'transform': 'rotate(180deg)'},
                    {'image': self._get_camera_image(common.last_camera_image), 'position': 'center', 'transform': 'rotate(180deg)'},
                    {'image': self._get_camera_image(common.last_camera_right_image), 'position': 'right'},
                ]
            )

        @app.route('/upload_image/', methods=['POST'])
        def upload_image():
            self.last_image = request.get_json()
            common.logger.info(f"/upload_image/ type={self.last_image.get('type', 'unknown')}")
            return jsonify({'status': 'ok'})

        @app.route('/custom_image/')
        def custom_image():
            if common.last_rosmap_image:
                image_data = self._get_camera_image(common.last_rosmap_image)
                return jsonify({'image': image_data})
            return jsonify(self.last_image)

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

    def _get_camera_image(self, msg):
        if not msg:
            return ""
        m = re.search(r" (.*) compressed", msg.format)
        return f"data:image/{m and m[1] or 'jpg'};base64,{base64.b64encode(msg.data).decode()}"

    def append_history(self, key, value):
        history = self.last_data[key]
        history.append(value)
        self.last_data[key] = history[-10:]

    def _track_event(self, event, data, emit=False):
        if isinstance(data, list) and len(data) == 1:
            data = data[0]
        try:
            data = json.loads(data)
        except Exception:
            pass

        if event == 'navigate' and data.get('type') == 'arrived':
            self.last_data['destination'].append('__arrived__')

        if event == 'share':
            if not isinstance(data, dict):
                common.logger.info(f"Unexpected share data {data}")
                return
            event_type = f"share.{data.get('type')}"
            # if not event_type.startswith('share.Speak'):
            #     common.logger.info(f"DEBUG {event_type} {payload}")
            value = data.get('value')

            if event_type == 'share.Speak':
                if isinstance(value, str) and value and not emit:
                    lst = self.last_data[event_type]
                    lst.append(value)
                    self.last_data[event_type] = lst[-10:]
                return

            if event_type == 'share.ChatStatus':
                if isinstance(value, str):
                    value = json.loads(value)
                    self.last_data[f'{event_type}.visible'] = [value.get('visible', False)]
                    messages = value.get('messages')
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
            self.last_data[event] = [data]
            return

        if event in self.SIMPLE_HISTORY_EVENTS:
            lst = self.last_data[event]
            lst.append(data)
            self.last_data[event] = lst[-10:]
            return

        if event not in self.IGNORE_EVENTS:
            common.logger.info(f"[IGNORE handle_event] event={event}, data={data}")
