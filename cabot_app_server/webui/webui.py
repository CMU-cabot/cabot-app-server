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
import math
import re
import socketio
from collections import defaultdict, deque
from datetime import datetime, timezone
from flask import Flask, Response, jsonify, render_template, request
from tf_transformations import euler_from_quaternion
import common
import tcp
import tour_manager


class WebUI:

    SIMPLE_LAST_EVENTS = {
        'cabot_name',
        'cabot_version',
        'elevator_settings',
        # 'device_status',
        # 'system_status',
        'battery_status',
        'location',
    }

    SIMPLE_HISTORY_EVENTS = {
        'touch',
        'manage_cabot',
        'navigate',
        'speak',
        'log',
    }

    TIMESTAMP_HISTORY_EVENTS = {
        'destination',
        'button',
    }

    IGNORE_EVENTS = {
        'req_version',
        'req_name',
        'heartbeat',
        'camera_image_request',
        'camera_image',
        'camera_orientation',
        'device_status',
        'system_status',
    }

    def __init__(self, server: tcp.CaBotTCP):
        app: Flask = server.app
        sio: socketio.Server = server.sio
        manage_cabot_char = server.manage_cabot_char
        cabot_manager = server.cabot_manager
        self.last_data = defaultdict(list)
        self.last_image = {}
        self.tour_manager = tour_manager.TourManager()
        self.location_buffer = deque(maxlen=60 * 60)

        @app.route('/_health/')
        def health():
            return jsonify({'status': 'ok'})

        @app.route('/')
        def index():
            return render_template("index.html")

        @app.route('/last_data/')
        def last_data():
            touch_buffer = list(common.touch_buffer)
            common.touch_buffer.clear()
            cmd_vel_buffer = list(common.cmd_vel_buffer)
            common.cmd_vel_buffer.clear()
            common.logger.info(f"touch_buffer: {len(touch_buffer)}, cmd_vel_buffer: {len(cmd_vel_buffer)}")
            self.last_data['average_touch'] = [sum(abs(obj.data) for obj in touch_buffer) / len(touch_buffer) if touch_buffer else -1]
            self.last_data['average_speed'] = [sum(abs(obj.linear.x) for obj in cmd_vel_buffer) / len(cmd_vel_buffer) if cmd_vel_buffer else -1]
            self.last_data['localize_status'] = [common.last_localize_status]
            self.last_data['device_status'] = [cabot_manager.device_status().json]
            self.last_data['system_status'] = [cabot_manager.cabot_system_status().json]
            msg, common.last_imu_data = common.last_imu_data, None
            if msg is None:
                self.last_data.pop('imu_data', None)
            else:
                roll, pitch, yaw = euler_from_quaternion(
                    (
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z,
                        msg.orientation.w,
                    )
                )
                self.last_data['imu_data'] = [{'roll': abs(math.degrees(roll)), 'pitch': abs(math.degrees(pitch))}]
            key = request.args.get('key')
            return jsonify({key: self.last_data.get(key, [])}) if key else jsonify(self.last_data)

        @app.route('/past_locations/')
        def past_locations():
            return jsonify(list(self.location_buffer))

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
            if data == 'restart_localization':
                common.last_localize_status = 0
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

        @app.route("/map/<path:path>", methods=["GET", "POST"])
        def proxy(path):
            import requests

            resp = requests.request(
                method=request.method,
                url=f"http://localhost:9090/map/{path}",
                params=request.args,
                headers={k: v for k, v in request.headers if k != "Host"},
                data=request.get_data(),
                cookies=request.cookies,
                stream=True,
            )
            EXCLUDED_HEADERS = {
                'content-length',
                'transfer-encoding',
                'content-encoding',
                'connection',
                'keep-alive',
                'proxy-authenticate',
                'proxy-authorization',
                'te',
                'trailers',
                'upgrade',
            }
            return Response(
                resp.content,
                status=resp.status_code,
                headers={k: v for k, v in resp.headers.items() if k.lower() not in EXCLUDED_HEADERS},
            )

        @app.route('/map/location-history.html')
        def location_history():
            return render_template("location-history.html")

        # Socket.IO Wrappers
        original_emit = sio.emit
        original_handler = sio._handle_event_internal
        original_event_char_callback = server.event_char.handleAnyEventCallback

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

        def event_char_wrap(request_id, event):
            if event.type == 'click':
                self._track_event(
                    'button',
                    {
                        'type': event.type,
                        'request_id': request_id,
                        'buttons': event.buttons,
                        'count': event.count,
                    },
                )
            elif event.type == 'holddown':
                self._track_event(
                    'button',
                    {
                        'type': event.type,
                        'request_id': request_id,
                        'holddown': event.holddown,
                        'duration': event.duration,
                    },
                )
            return original_event_char_callback(request_id, event)

        sio.emit = emit_wrap
        sio._handle_event_internal = handler_wrap
        server.event_char.handleAnyEventCallback = event_char_wrap

        self.tour_manager.load()
        common.logger.info("WebUI listening...")

    def _get_camera_image(self, msg):
        if not msg:
            return ""
        m = re.search(r" (.*) compressed", msg.format)
        return f"data:image/{m and m[1] or 'jpg'};base64,{base64.b64encode(msg.data).decode()}"

    def _track_event(self, event, data, emit=False):
        if isinstance(data, list) and len(data) == 1:
            data = data[0]
        try:
            data = json.loads(data)
        except Exception:
            pass

        if event == 'navigate' and data.get('type') == 'arrived':
            # self.last_data['destination'].append('__arrived__')
            self.last_data['destination'].append({'timestamp': datetime.now(timezone.utc).isoformat(), 'data': '__arrived__'})

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
                    # lst.append(value)
                    lst.append({'timestamp': datetime.now(timezone.utc).isoformat(), 'data': value})
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
            if event == 'location':
                self.location_buffer.append(data)
            return

        if event in self.SIMPLE_HISTORY_EVENTS:
            lst = self.last_data[event]
            lst.append(data)
            self.last_data[event] = lst[-10:]
            return

        if event in self.TIMESTAMP_HISTORY_EVENTS:
            lst = self.last_data[event]
            lst.append({'timestamp': datetime.now(timezone.utc).isoformat(), 'data': data})
            self.last_data[event] = lst[-10:]
            return

        if event not in self.IGNORE_EVENTS:
            common.logger.info(f"[IGNORE handle_event] event={event}, data={data}")
            self.last_data[f'___{event}___'] = [data]


common.last_localize_status = 2  # force reset
