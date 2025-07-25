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
import codecs

DEBUG = False

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

file_directory = "/opt/cabot/docker/home/.ros/log/"

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
            # TODO: get title and detail by log_name
            log_name = request["log_name"]
            (is_report_submitted, is_uploaded_to_box, title, detail) = self.getReport(log_name)
            response["log"] = {
                "name": log_name,
                "title": title,
                "detail": detail,
                "is_report_submitted": is_report_submitted == "1",
                "is_uploaded_to_box": is_uploaded_to_box == "1"
            }
        elif request_type == "report":
            title = request["title"]
            detail = request["detail"]
            name = request["log_name"]
            duration = self.getDuration(name)
            self.makeReport(title, detail, name)
            response["log"] = {
                "name": name,
                "nanoseconds": int(duration)
            }
        elif request_type == "data-chunk":
            chunk_data = request["data"]
            app_log_name = request["appLogName"]
            chunk_index = request["chunkIndex"]
            cabot_log_name = request["cabotLogName"]
            logger.info(f"chunk index of this queue is {chunk_index}")
            file_path = file_directory + cabot_log_name + "/" + app_log_name

            if app_log_name not in self.data_chunks:
                self.data_chunks[app_log_name] = {
                    "chunks": {},   # {chunk_index : binary_data}
                    "count": 0,
                    "decoder": codecs.getincrementaldecoder("utf-8")()
                }
                
                with open(file_path, "w") as f:
                    pass    # empt

            binary_data = base64.b64decode(chunk_data)
            chunk_name = self.data_chunks[app_log_name]
            if chunk_name["count"] == chunk_index:
                self.write_text_with_decoder(file_path, binary_data, chunk_name["decoder"], False)

                chunk_name["count"] += 1
                while chunk_name["count"] in chunk_name["chunks"]:
                    chunk_count = chunk_name["count"]
                    self.write_text_with_decoder(file_path, chunk_name["chunks"][chunk_count], chunk_name["decoder"], False)
                    chunk_name["count"] += 1
            elif chunk_name["count"] < chunk_index:
                chunk_name["chunks"][chunk_index] = binary_data

            return
        elif request_type == "appLog":
            app_log_name = request["appLogName"]
            cabot_log_name = request["cabotLogName"]
            total_chunks = request["totalChunks"]

            if app_log_name in self.data_chunks:
                data_info = self.data_chunks[app_log_name]

                if data_info["count"] == total_chunks:
                    file_path = file_directory + cabot_log_name + "/" + app_log_name
                    logger.info(f"app log save to {file_path}")
                    self.write_text_with_decoder(file_path, b"", data_info["decoder"], True)

                    del self.data_chunks[app_log_name]
                    self.submitReport()
                else:
                    self.add_to_queue(request_json, callback)
            return

        callback(response)

    def add_to_queue(self, request_json, callback, output=True):
        if output:
            logger.info(f"add to queue {request_json}")
        self.request_queue.put((request_json, callback))

    def write_text_with_decoder(self, file_path, data, decoder, final):
        text_part = decoder.decode(data, final=final)
        with open(file_path, "a") as f:
            f.write(text_part)
