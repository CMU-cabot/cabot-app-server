import json

from flask import Flask, jsonify, render_template, request
import socketio
import common


class WebUI:

    def __init__(self, app: Flask, sio: socketio.Server):
        common.logger.info("WebUI listening...")

        @app.route('/')
        def index():
            return render_template("index.html")

        @app.route('/publish/')
        def publish():
            event = request.args.get('event')
            raw_data = request.args.get('data')
            if not event:
                return jsonify({'error': 'event parameter is required'}), 400
            if raw_data is None:
                return jsonify({'error': 'data parameter is required'}), 400
            try:
                payload = json.loads(raw_data)
            except json.JSONDecodeError as exc:
                common.logger.error(f"/publish invalid json payload: {exc}")
                return jsonify({'error': 'invalid json payload', 'detail': str(exc)}), 400

            target_sid = request.args.get('to')
            emit_kwargs = {'to': target_sid} if target_sid else {}
            common.logger.info(f"/publish event={event}, to={target_sid}, payload={payload}")
            sio.emit(event, payload, **emit_kwargs)
            return jsonify({'status': 'ok'})

        _original_emit = sio.emit
        _original_handler = sio._handle_event_internal

        def emit_wrap(event, data=None, to=None, **kw):
            # if event not in ['touch']:
            #     common.logger.info(f"[OUT] event={event}, data={data}, to={to}")
            return _original_emit(event, data=data, to=to, **kw)

        def handler_wrap(server, sid, eio_sid, raw_data, namespace, id):
            try:
                if isinstance(raw_data, list) and len(raw_data) == 2:
                    event = raw_data[0]
                    payload = raw_data[1]
                    if isinstance(payload, list) and len(payload) == 1:
                        payload = payload[0]
                    if event == 'share' and isinstance(payload, str):
                        payload = json.loads(payload)
                        type = payload.get('type')
                        value = payload.get('value')
                        if type in [
                            'Speak',
                            'ChangeLanguage',
                            'ChatStatus',
                            'PossibleHandleSide',
                            'ChangeHandleSide',
                            'PossibleTouchMode',
                            'ChangeTouchMode',
                            'ChangeUserVoiceRate',
                            'ChangeUserVoiceType',
                            'Tour',
                            'OverrideTour',
                            'OverrideDestination',
                        ]:
                            common.logger.info(f"[TRACK] {type} {value}")
                        else:
                            common.logger.info(f"[IGNORE] event={event}, type={type}, data={payload}")
                    # else:
                    #     common.logger.info(f"[IGNORE] event={event}, data={payload}")
            except Exception as e:
                common.logger.error(f"handler_wrap error: {e}")
            return _original_handler(server, sid, eio_sid, raw_data, namespace, id)

        sio.emit = emit_wrap
        sio._handle_event_internal = handler_wrap
