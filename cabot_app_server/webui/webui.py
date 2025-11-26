from flask import Flask, render_template
import socketio
import common


class WebUI():

    def __init__(self, app: Flask, sio: socketio.Server):
        common.logger.info("WebUI listening...")

        @app.route('/')
        def index():
            return render_template("index.html")

        _original_emit = sio.emit
        _original_handler = sio._handle_event_internal

        def emit_wrap(event, data=None, to=None, **kw):
            common.logger.info(f"[OUT] event={event}, data={data}, to={to}")
            return _original_emit(event, data=data, to=to, **kw)

        def handler_wrap(server, sid, eio_sid, data, namespace, id):
            try:
                if isinstance(data, list) and len(data) > 0:
                    event = data[0]
                    payload = data[1] if len(data) > 1 else None
                else:
                    event = None
                    payload = data
                common.logger.info(f"[IN] event={event}, data={payload}")
            except Exception as e:
                common.logger.error(f"handler_wrap error: {e}")
            return _original_handler(server, sid, eio_sid, data, namespace, id)

        sio.emit = emit_wrap
        sio._handle_event_internal = handler_wrap
