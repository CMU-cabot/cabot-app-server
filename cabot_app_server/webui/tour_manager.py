import threading
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
        self.features = {}
        self.tour_data = {}
        self.directory = {}

    def background_task(self):
        self.features = {}
        self.tour_data = {}
        self.directory = {}
        self.config = requests.get(self.CONFIG_URL).json()
        common.logger.info(f"config={self.config}")
        initial_location = self.config.get('INITIAL_LOCATION', {})
        dist = self.config.get('MAX_RADIUS')
        lat = initial_location.get('lat')
        lng = initial_location.get('lng')
        if dist and lat and lng:
            self.landmarks = requests.get(self.START_URL.format(lat=lat, lng=lng, dist=dist)).json()
            self.features = requests.get(self.FEATURES_URL).json()
            common.logger.info(f"{len(self.features)} features")
            self.tour_data = requests.get(self.TOURDATA_URL).json()
            common.logger.info(f"{len(self.tour_data.get('tours', []))} tours, {len(self.tour_data.get('destinations', []))} destinations")
            for lang in ['ja', 'en', 'zh-CN']:
                directory = requests.get(self.DIRECTORY_URL.format(lat=lat, lng=lng, dist=dist, lang=lang)).json()
                common.logger.info(f"lang={lang}, {len(directory.get('landmarks', []))} landmarks, {len(directory.get('sections', []))} sections")
                self.directory[lang] = directory

    def load(self):
        threading.Thread(target=self.background_task).start()

    def format_directories(self):
        return {'sections': {lang: item.get('sections', []) for lang, item in self.directory.items()}} | {'features': self.features} | {k: v for k, v in self.tour_data.items()}
