#!/usr/bin/env python

# Copyright (c) 2023  Carnegie Mellon University and Miraikan
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

import json
import logging
import os
import queue
import subprocess
import threading
import time
import base64
import binascii
import codecs
import re
import shutil

DEBUG = False

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

file_directory = "/opt/cabot/docker/home/.ros/log/"
attachment_directory_name = "mobile_attachments"
manifest_filename = "manifest.json"
webui_reports_directory_name = "webui_reports"
report_id_pattern = re.compile(r"^[A-Za-z0-9_-]+$")

class LogReport:
    def __init__(self):
        self.request_queue = queue.Queue()
        self.data_chunks = {}

        thread = threading.Thread(target=self.observer)
        thread.setDaemon(True)
        thread.start()

    def observer(self):
        while True:
            while self.request_queue.empty():
                time.sleep(0.1)

            (request_json, callback) = self.request_queue.get()
            self.response_log(request_json, callback)

            self.request_queue.task_done()

    def canUploadReport(self):
        command = ["sudo", "-E", "/opt/report-submitter/can_upload_report.sh"]
        result = subprocess.run(command, capture_output=True, text=True, env=os.environ.copy()).returncode
        logger.info(f"{result}, {result==0}")
        return result == 0

    def getLogList(self):
        command = ["sudo", "-E", "/opt/report-submitter/get_log_list.sh"]
        result = subprocess.run(command, capture_output=True, text=True, env=os.environ.copy()).stdout
        return result.split()

    def makeReport(self, title, detail, name):
        command = ["sudo", "-E", "/opt/report-submitter/create_list.sh", title, detail, name]
        result = subprocess.run(command, capture_output=True, text=True, env=os.environ.copy()).stdout
        return result.split()

    def makeWebuiReport(self, report_id, title, detail, name, submit=True):
        command = [
            "sudo", "-E", "/opt/report-submitter/create_webui_issue.sh",
            title, detail, name, report_id, "1" if submit else "0"
        ]
        result = subprocess.run(command, capture_output=True, text=True, env=os.environ.copy())
        if result.returncode != 0:
            message = result.stderr.strip() or result.stdout.strip() or "failed to create webui issue entry"
            raise RuntimeError(message)
        return result.stdout.split()

    def submitReport(self):
        command = ["systemctl", "--user", "start", "submit_report"]
        subprocess.run(command, capture_output=True, text=True, env=os.environ.copy()).stdout

    def getReport(self, name):
        command = ["sudo", "-E", "/opt/report-submitter/get_report.sh", name]
        result = subprocess.run(command, capture_output=True, text=True, env=os.environ.copy()).stdout
        items = result.split("\n")
        if len(items) >= 4:
            return (items[0], items[1], items[2], "\n".join(items[3:]))
        return (0, 0, "", "")

    def getDuration(self, name):
        command = ["sudo", "-E", "/opt/report-submitter/get_duration.sh", name]
        result = subprocess.run(command, capture_output=True, text=True, env=os.environ.copy()).stdout
        if not result:
            result = 0
        return result

    def log_directory(self, cabot_log_name):
        return os.path.join(file_directory, cabot_log_name)

    def attachment_directory(self, cabot_log_name):
        return os.path.join(self.log_directory(cabot_log_name), attachment_directory_name)

    def manifest_path(self, cabot_log_name):
        return os.path.join(self.attachment_directory(cabot_log_name), manifest_filename)

    def latest_log_name(self):
        latest_path = os.path.join(file_directory, "latest")
        resolved_path = os.path.realpath(latest_path)
        file_root = os.path.realpath(file_directory)
        if not resolved_path or not os.path.isdir(resolved_path):
            raise FileNotFoundError("active log directory is not available")
        if os.path.commonpath([file_root, resolved_path]) != file_root:
            raise FileNotFoundError("active log directory is invalid")
        log_name = os.path.basename(resolved_path.rstrip("/"))
        if not log_name.startswith("cabot_"):
            raise FileNotFoundError("active log directory is invalid")
        return log_name

    def validate_log_name(self, log_name):
        normalized = os.path.basename(str(log_name).strip())
        if normalized != log_name:
            raise ValueError("log_name is invalid")
        if not normalized.startswith("cabot_"):
            raise ValueError("log_name is invalid")

        log_directory = self.log_directory(normalized)
        if not os.path.isdir(log_directory):
            raise FileNotFoundError("log directory is not available")
        return normalized

    def webui_report_directory(self, cabot_log_name, report_id):
        return os.path.join(self.log_directory(cabot_log_name), webui_reports_directory_name, report_id)

    def webui_manifest_path(self, cabot_log_name, report_id):
        return os.path.join(self.webui_report_directory(cabot_log_name, report_id), manifest_filename)

    def save_webui_manifest(self, cabot_log_name, report_id, attachments):
        report_directory = self.webui_report_directory(cabot_log_name, report_id)
        os.makedirs(report_directory, exist_ok=True)
        with open(self.webui_manifest_path(cabot_log_name, report_id), "w") as manifest_file:
            json.dump({"attachments": attachments}, manifest_file)

    def save_webui_attachment(self, cabot_log_name, report_id, attachment):
        report_directory = self.webui_report_directory(cabot_log_name, report_id)
        os.makedirs(report_directory, exist_ok=True)
        file_name = os.path.basename(str(attachment.get("file_name", "")).strip())
        if not file_name:
            raise ValueError("attachment file name is required")
        raw = attachment.get("data", "")
        file_path = os.path.join(report_directory, file_name)
        try:
            decoded = base64.b64decode(raw, validate=True)
        except (ValueError, binascii.Error) as exc:
            raise ValueError("attachment data is invalid") from exc
        with open(file_path, "wb") as output_file:
            output_file.write(decoded)
        return {
            "file_name": file_name,
            "original_name": str(attachment.get("original_name", file_name)),
            "order": int(attachment.get("order", 1)),
        }

    def create_webui_issue_entry(self, report_id, title, detail, log_name, submit=True):
        self.makeWebuiReport(report_id, title, detail, log_name, submit)

    def reset_webui_report_directory(self, cabot_log_name, report_id):
        report_directory = self.webui_report_directory(cabot_log_name, report_id)
        if os.path.isdir(report_directory):
            shutil.rmtree(report_directory)

    def normalize_webui_attachments(self, attachments):
        source = attachments or []
        if not isinstance(source, list):
            raise ValueError("attachments must be a list")

        normalized = []
        for index, attachment in enumerate(source, start=1):
            if not isinstance(attachment, dict):
                raise ValueError(f"attachment {index} is invalid")

            file_name = os.path.basename(str(attachment.get("file_name", "")).strip()) or f"webui-attachment-{index}.png"
            original_name = str(attachment.get("original_name", file_name)).strip() or file_name
            raw_order = attachment.get("order", index)
            try:
                order = int(raw_order)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"attachment {index} order is invalid") from exc

            data = str(attachment.get("data", ""))
            if not data:
                raise ValueError(f"attachment {index} data is invalid")

            normalized.append({
                "file_name": file_name,
                "original_name": original_name,
                "order": order,
                "data": data,
            })

        return normalized

    def create_webui_report(self, report_id, title, detail, log_name, attachments=None, submit=True):
        if not report_id:
            raise ValueError("report_id is required")
        if not title:
            raise ValueError("title is required")
        if not log_name:
            raise ValueError("log_name is required")
        if not report_id_pattern.match(report_id):
            raise ValueError("report_id is invalid")
        log_name = self.validate_log_name(log_name)
        normalized_attachments = self.normalize_webui_attachments(attachments)
        self.reset_webui_report_directory(log_name, report_id)
        saved_attachments = []
        for attachment in normalized_attachments:
            saved_attachments.append(self.save_webui_attachment(log_name, report_id, attachment))
        if saved_attachments:
            self.save_webui_manifest(log_name, report_id, saved_attachments)
        self.create_webui_issue_entry(report_id, title, detail, log_name, submit)
        if submit:
            self.submitReport()
        return {
            "report_id": report_id,
            "log_name": log_name,
            "attachment_count": len(saved_attachments),
            "submitted": bool(submit),
        }

    def normalize_attachments(self, attachments):
        normalized = []
        source = attachments or []
        source = sorted(source, key=lambda item: (int(item.get("order", 0)), item.get("file_name", "")))
        for index, attachment in enumerate(source, start=1):
            file_name = os.path.basename(str(attachment.get("file_name", "")).strip())
            if not file_name:
                continue
            original_name = str(attachment.get("original_name", file_name))
            normalized.append({
                "file_name": file_name,
                "original_name": original_name,
                "order": index
            })
        return normalized

    def load_manifest(self, cabot_log_name):
        manifest_path = self.manifest_path(cabot_log_name)
        if not os.path.isfile(manifest_path):
            return {"attachments": []}
        try:
            with open(manifest_path, "r") as manifest_file:
                return json.load(manifest_file)
        except Exception as exc:
            logger.error(f"failed to load manifest {manifest_path}: {exc}")
            return {"attachments": []}

    def save_manifest(self, cabot_log_name, attachments):
        os.makedirs(self.attachment_directory(cabot_log_name), exist_ok=True)
        manifest_path = self.manifest_path(cabot_log_name)
        with open(manifest_path, "w") as manifest_file:
            json.dump({"attachments": attachments}, manifest_file)

    def missing_attachment_names(self, cabot_log_name, attachments):
        return [
            attachment["file_name"]
            for attachment in attachments
            if not os.path.isfile(os.path.join(self.attachment_directory(cabot_log_name), attachment["file_name"]))
        ]

    def load_attachment_details(self, cabot_log_name):
        manifest = self.load_manifest(cabot_log_name)
        attachments = []
        for attachment in manifest.get("attachments", []):
            file_path = os.path.join(self.attachment_directory(cabot_log_name), attachment["file_name"])
            if not os.path.isfile(file_path):
                continue
            with open(file_path, "rb") as attachment_file:
                attachment_data = base64.b64encode(attachment_file.read()).decode("utf-8")
            response_attachment = dict(attachment)
            response_attachment["image_data"] = attachment_data
            attachments.append(response_attachment)
        return attachments

    def asset_key(self, cabot_log_name, file_name, asset_type):
        return f"{cabot_log_name}:{asset_type}:{file_name}"

    def response_log(self, request_json, callback):
        try:
            request = json.loads(request_json)
        except json.JSONDecodeError as e:
            logger.error(f"json cannot be parsed {e}")
            return

        if "type" not in request:
            logger.error(f"not type in the request {request}")
            return

        request_type = request["type"]
        response = {
            "response_id": time.clock_gettime_ns(time.CLOCK_REALTIME),
            "type": request_type
        }
        if request_type == "list":
            response["status"] = "NG" if self.canUploadReport() else "OK"
            response["log_list"] = []
            log_names = self.getLogList()
            for log_name in log_names:
                items = log_name.split(",")
                is_report_submitted = (items[1] == "1") if len(items) > 1 else False
                is_uploaded_to_box = (items[2] == "1") if len(items) > 2 else False
                nanoseconds = int(items[3]) if len(items) > 3 else None
                response["log_list"].append({
                    "name": items[0],
                    "nanoseconds": nanoseconds,
                    "is_report_submitted": is_report_submitted,
                    "is_uploaded_to_box": is_uploaded_to_box
                })
        elif request_type == "detail":
            log_name = request["log_name"]
            (is_report_submitted, is_uploaded_to_box, title, detail) = self.getReport(log_name)
            response["log"] = {
                "name": log_name,
                "title": title,
                "detail": detail,
                "is_report_submitted": is_report_submitted == "1",
                "is_uploaded_to_box": is_uploaded_to_box == "1",
                "attachments": self.load_attachment_details(log_name)
            }
        elif request_type == "report":
            title = request["title"]
            detail = request["detail"]
            name = request["log_name"]
            duration = self.getDuration(name)
            attachments = self.normalize_attachments(request.get("attachments", []))
            self.makeReport(title, detail, name)
            self.save_manifest(name, attachments)
            response["log"] = {
                "name": name,
                "nanoseconds": int(duration),
                "missing_attachment_names": self.missing_attachment_names(name, attachments)
            }
        elif request_type == "data-chunk":
            chunk_data = request["data"]
            file_name = request.get("fileName") or request.get("appLogName")
            asset_type = request.get("fileType", "appLog")
            chunk_index = request["chunkIndex"]
            cabot_log_name = request["cabotLogName"]
            if not file_name:
                logger.error(f"file name is missing in chunk request {request}")
                return
            logger.info(f"chunk index of this queue is {chunk_index}")
            safe_file_name = os.path.basename(file_name)
            if asset_type == "attachmentImage":
                os.makedirs(self.attachment_directory(cabot_log_name), exist_ok=True)
                file_path = os.path.join(self.attachment_directory(cabot_log_name), safe_file_name)
            else:
                file_path = os.path.join(self.log_directory(cabot_log_name), safe_file_name)
            asset_key = self.asset_key(cabot_log_name, file_name, asset_type)

            if asset_key not in self.data_chunks:
                self.data_chunks[asset_key] = {
                    "chunks": {},   # {chunk_index : binary_data}
                    "count": 0,
                    "decoder": None if asset_type == "attachmentImage" else codecs.getincrementaldecoder("utf-8")(),
                    "is_binary": asset_type == "attachmentImage"
                }

                with open(file_path, "wb" if asset_type == "attachmentImage" else "w") as f:
                    pass

            binary_data = base64.b64decode(chunk_data)
            data_info = self.data_chunks[asset_key]
            if data_info["count"] == chunk_index:
                self.write_asset_chunk(file_path, binary_data, data_info, False)

                data_info["count"] += 1
                while data_info["count"] in data_info["chunks"]:
                    chunk_count = data_info["count"]
                    self.write_asset_chunk(file_path, data_info["chunks"][chunk_count], data_info, False)
                    del data_info["chunks"][chunk_count]
                    data_info["count"] += 1
            elif data_info["count"] < chunk_index:
                data_info["chunks"][chunk_index] = binary_data

            return
        elif request_type == "appLog":
            file_name = request.get("fileName") or request.get("appLogName")
            cabot_log_name = request["cabotLogName"]
            asset_type = request.get("fileType", "appLog")
            total_chunks = request["totalChunks"]
            is_last_file = request.get("isLastFile", False)

            if not file_name:
                if is_last_file:
                    self.submitReport()
                return

            asset_key = self.asset_key(cabot_log_name, file_name, asset_type)
            if asset_key in self.data_chunks:
                data_info = self.data_chunks[asset_key]
                if data_info["count"] == total_chunks:
                    safe_file_name = os.path.basename(file_name)
                    if asset_type == "attachmentImage":
                        file_path = os.path.join(self.attachment_directory(cabot_log_name), safe_file_name)
                    else:
                        file_path = os.path.join(self.log_directory(cabot_log_name), safe_file_name)
                    logger.info(f"asset save to {file_path}")
                    self.write_asset_chunk(file_path, b"", data_info, True)

                    del self.data_chunks[asset_key]
                    if is_last_file:
                        self.submitReport()
                else:
                    self.add_to_queue(request_json, callback)
            return

        callback(response)

    def add_to_queue(self, request_json, callback, output=True):
        if output:
            logger.info(f"add to queue {request_json}")
        self.request_queue.put((request_json, callback))

    def write_asset_chunk(self, file_path, data, data_info, final):
        if data_info["is_binary"]:
            if data:
                with open(file_path, "ab") as f:
                    f.write(data)
            return

        text_part = data_info["decoder"].decode(data, final=final)
        with open(file_path, "a") as f:
            f.write(text_part)
