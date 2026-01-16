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

import threading
import time
import requests
import json
import common


class TourManager:

    CONFIG_URL = 'http://localhost:9090/map/api/config'
    START_URL = 'http://localhost:9090/map/routesearch?action=start&lat={lat}&lng={lng}&dist={dist}&user=cabot'
    FEATURES_URL = 'http://localhost:9090/map/routesearch?action=features&user=cabot'
    TOURDATA_URL = 'http://localhost:9090/map/cabot/tourdata.json'
    DIRECTORY_URL = 'http://localhost:9090/query/directory?user=cabot&lat={lat}&lng={lng}&dist={dist}&lang={lang}'

    def __init__(self):
        self.features = []
        self.tour_data = {}
        self.directory = {}

    def background_task(self):
        self.features = []
        self.tour_data = {}
        self.directory = {}
        for retry in range(60):
            try:
                self.config = requests.get(self.CONFIG_URL).json()
                common.logger.info(f"config={self.config}")
                initial_location = self.config.get('INITIAL_LOCATION', {})
                dist = self.config.get('MAX_RADIUS')
                lat = initial_location.get('lat')
                lng = initial_location.get('lng')
                if dist and lat and lng:
                    self.landmarks = requests.get(self.START_URL.format(lat=lat, lng=lng, dist=dist)).json()
                    self.features = requests.get(self.FEATURES_URL).json()
                    if self.features:
                        common.logger.info(f"{len(self.features)} features")
                        self.tour_data = requests.get(self.TOURDATA_URL).json()
                        common.logger.info(f"{len(self.tour_data.get('tours', []))} tours, {len(self.tour_data.get('destinations', []))} destinations")
                        for lang in ['ja', 'en', 'zh-CN']:
                            directory = requests.get(self.DIRECTORY_URL.format(lat=lat, lng=lng, dist=dist, lang=lang)).json()
                            common.logger.info(f"lang={lang}, {len(directory.get('landmarks', []))} landmarks, {len(directory.get('sections', []))} sections")
                            self.directory[lang] = directory
                        return
            except Exception as e:
                common.logger.warning(f"TourManager background_task retry {retry}: {e}")
            time.sleep(5)

    def load(self):
        threading.Thread(target=self.background_task).start()

    def format_directories(self):
        return (
            {'sections': {lang: item.get('sections', []) for lang, item in self.directory.items()}}
            | {'node_names': self.build_node_names(self.features)}
            | {'destinations': self.tour_data.get('destinations')}
            | {'tours': self.tour_data.get('tours')}
        )

    def build_node_names(self, features):
        result = {}
        for lang in ['ja', 'en', 'zh-CN']:
            node_names = {}
            for feature in features:
                p = feature.get("properties") or {}
                if p.get("facil_id"):
                    name = p.get(f"name_{lang}")
                    if name:
                        for i in range(1, 10):
                            node = p.get(f"ent{i}_node")
                            if node:
                                ent_name = p.get(f"ent{i}_n")
                                if ent_name:
                                    node_names[node] = f"{name} {ent_name}"
                                else:
                                    node_names[node] = name

            result[lang] = node_names
        return result
